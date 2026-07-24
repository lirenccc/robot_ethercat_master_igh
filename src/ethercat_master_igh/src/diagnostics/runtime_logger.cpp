/**
 * @file runtime_logger.cpp
 * @brief 运行时诊断日志实现
 */

#include "ethercat_joint/diagnostics/runtime_logger.hpp"
#include "ethercat_joint/servo/cia402.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <sys/time.h>

namespace ethercat_joint {

uint64_t getMonotonicTimeNs()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
           static_cast<uint64_t>(ts.tv_nsec);
}

const char* RuntimeLogger::eventTypeName(RuntimeLogEventType type)
{
    switch (type) {
        case RuntimeLogEventType::CIA402_STATE: return "cia402_state";
        case RuntimeLogEventType::CONTROL_WORD: return "control_word";
        case RuntimeLogEventType::SERVO_COMMAND: return "servo_command";
        case RuntimeLogEventType::ERROR_CODE: return "error_code";
        case RuntimeLogEventType::TARGET_SETPOINT: return "target_setpoint";
        default: return "unknown";
    }
}

const char* RuntimeLogger::cia402StateName(uint16_t state)
{
    return ethercat_joint::cia402StateName(state);
}

RuntimeLogger::~RuntimeLogger()
{
    shutdown();
}

bool RuntimeLogger::initialize(const std::string& log_dir)
{
    shutdown();

    std::error_code ec;
    std::filesystem::create_directories(log_dir, ec);
    if (ec) {
        return false;
    }

    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
    localtime_r(&t, &tm_buf);

    std::ostringstream filename;
    filename << log_dir << "/runtime_"
             << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << ".csv";

    log_file_.open(filename.str(), std::ios::out | std::ios::trunc);
    if (!log_file_.is_open()) {
        return false;
    }

    log_dir_ = log_dir;
    log_file_path_ = filename.str();
    mono_base_ns_ = getMonotonicTimeNs();
    wall_base_time_ = std::chrono::system_clock::now();
    ring_head_.store(0, std::memory_order_relaxed);
    ring_tail_.store(0, std::memory_order_relaxed);

    log_file_ << "iso_time,elapsed_ns,event_type,motor_id,status_word,cia402_state,cia402_state_name,"
                 "control_word,operation_mode,detail,target_position,target_velocity,target_torque,error_code\n";
    log_file_.flush();

    enabled_.store(true, std::memory_order_release);
    return true;
}

void RuntimeLogger::shutdown()
{
    if (!enabled_.load(std::memory_order_acquire)) {
        return;
    }

    drain(nullptr);

    {
        std::lock_guard<std::mutex> lock(file_mutex_);
        if (log_file_.is_open()) {
            log_file_.flush();
            log_file_.close();
        }
    }

    enabled_.store(false, std::memory_order_release);
}

bool RuntimeLogger::tryPushToRing(const RuntimeLogEvent& event)
{
    const size_t head = ring_head_.load(std::memory_order_relaxed);
    const size_t next = (head + 1) % kRingCapacity;
    if (next == ring_tail_.load(std::memory_order_acquire)) {
        return false;
    }
    ring_buffer_[head] = event;
    ring_head_.store(next, std::memory_order_release);
    return true;
}

bool RuntimeLogger::tryPopFromRing(RuntimeLogEvent& event)
{
    const size_t tail = ring_tail_.load(std::memory_order_relaxed);
    if (tail == ring_head_.load(std::memory_order_acquire)) {
        return false;
    }
    event = ring_buffer_[tail];
    ring_tail_.store((tail + 1) % kRingCapacity, std::memory_order_release);
    return true;
}

void RuntimeLogger::pushRtEvent(const RuntimeLogEvent& event)
{
    if (!enabled_.load(std::memory_order_acquire)) {
        return;
    }
    tryPushToRing(event);
}

void RuntimeLogger::queueCommand(const RuntimeLogEvent& event)
{
    if (!enabled_.load(std::memory_order_acquire)) {
        return;
    }
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_commands_.push_back(event);
}

void RuntimeLogger::flushPendingToBuffer()
{
    if (!enabled_.load(std::memory_order_acquire)) {
        return;
    }

    std::deque<RuntimeLogEvent> local_pending;
    {
        std::unique_lock<std::mutex> lock(pending_mutex_, std::try_to_lock);
        if (!lock) {
            return;
        }
        local_pending.swap(pending_commands_);
    }

    for (const auto& event : local_pending) {
        tryPushToRing(event);
    }
}

std::string RuntimeLogger::formatIsoTimestamp(uint64_t monotonic_ns) const
{
    const auto delta = std::chrono::nanoseconds(
        static_cast<int64_t>(monotonic_ns - mono_base_ns_));
    const auto wall = wall_base_time_ + delta;

    const std::time_t t = std::chrono::system_clock::to_time_t(wall);
    std::tm tm_buf{};
    localtime_r(&t, &tm_buf);

    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        wall.time_since_epoch()) % 1000;

    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S")
        << '.' << std::setw(3) << std::setfill('0') << ms.count();
    return oss.str();
}

void RuntimeLogger::writeEvent(const RuntimeLogEvent& event)
{
    const uint64_t elapsed_ns = event.timestamp_ns >= mono_base_ns_
        ? event.timestamp_ns - mono_base_ns_ : 0;

    log_file_ << formatIsoTimestamp(event.timestamp_ns) << ','
              << elapsed_ns << ','
              << eventTypeName(event.type) << ','
              << static_cast<unsigned>(event.motor_id) << ','
              << "0x" << std::hex << std::setw(4) << std::setfill('0') << event.status_word << std::dec << ','
              << "0x" << std::hex << std::setw(2) << std::setfill('0') << event.cia402_state << std::dec << ','
              << cia402StateName(event.cia402_state) << ','
              << "0x" << std::hex << std::setw(4) << std::setfill('0') << event.control_word << std::dec << ','
              << static_cast<int>(event.operation_mode) << ','
              << '"' << event.detail << '"' << ','
              << event.target_position << ','
              << event.target_velocity << ','
              << event.target_torque << ','
              << "0x" << std::hex << std::setw(4) << std::setfill('0') << event.error_code << std::dec
              << '\n';
}

size_t RuntimeLogger::drain(std::function<uint16_t(uint8_t motor_id)> error_code_fetcher)
{
    if (!enabled_.load(std::memory_order_acquire)) {
        return 0;
    }

    size_t count = 0;
    RuntimeLogEvent event;

    std::lock_guard<std::mutex> lock(file_mutex_);
    if (!log_file_.is_open()) {
        return 0;
    }

    while (tryPopFromRing(event)) {
        if (event.type == RuntimeLogEventType::ERROR_CODE && event.error_code == 0 && error_code_fetcher) {
            event.error_code = error_code_fetcher(event.motor_id);
        }
        writeEvent(event);
        ++count;
    }

    if (count > 0) {
        log_file_.flush();
    }
    return count;
}

}  // namespace ethercat_joint
