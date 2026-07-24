/**
 * @file runtime_logger.hpp
 * @brief 运行时诊断日志：CiA402 状态、伺服命令、错误码
 */

#ifndef RUNTIME_LOGGER_HPP
#define RUNTIME_LOGGER_HPP

#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>

namespace ethercat_joint {

enum class RuntimeLogEventType : uint8_t {
    CIA402_STATE = 0,
    CONTROL_WORD = 1,
    SERVO_COMMAND = 2,
    ERROR_CODE = 3,
    TARGET_SETPOINT = 4,
};

struct RuntimeLogEvent {
    uint64_t timestamp_ns = 0;
    uint8_t motor_id = 0;
    RuntimeLogEventType type = RuntimeLogEventType::CIA402_STATE;
    uint16_t status_word = 0;
    uint16_t control_word = 0;
    uint16_t cia402_state = 0;
    int8_t operation_mode = 0;
    int32_t target_position = 0;
    int32_t target_velocity = 0;
    int16_t target_torque = 0;
    uint16_t error_code = 0;
    char detail[256] = {};
};

class RuntimeLogger {
public:
    static constexpr size_t kRingCapacity = 2048;

    RuntimeLogger() = default;
    ~RuntimeLogger();

    bool initialize(const std::string& log_dir);
    void shutdown();

    bool isEnabled() const { return enabled_; }
    const std::string& logFilePath() const { return log_file_path_; }

    // RT 线程安全：写入环形缓冲
    void pushRtEvent(const RuntimeLogEvent& event);

    // 非 RT 线程：将待处理命令合并到环形缓冲
    void queueCommand(const RuntimeLogEvent& event);

    // RT 线程：非阻塞地将待处理命令转入环形缓冲
    void flushPendingToBuffer();

    // 非 RT 线程：刷写文件；fault 事件可通过 fetcher 读取 0x603F
    size_t drain(std::function<uint16_t(uint8_t motor_id)> error_code_fetcher = nullptr);

    static const char* eventTypeName(RuntimeLogEventType type);
    static const char* cia402StateName(uint16_t state);

private:
    bool tryPushToRing(const RuntimeLogEvent& event);
    bool tryPopFromRing(RuntimeLogEvent& event);
    void writeEvent(const RuntimeLogEvent& event);
    std::string formatIsoTimestamp(uint64_t monotonic_ns) const;

    std::atomic<bool> enabled_{false};
    std::string log_dir_;
    std::string log_file_path_;
    uint64_t mono_base_ns_ = 0;
    std::chrono::system_clock::time_point wall_base_time_{};

    std::array<RuntimeLogEvent, kRingCapacity> ring_buffer_{};
    std::atomic<size_t> ring_head_{0};
    std::atomic<size_t> ring_tail_{0};

    std::deque<RuntimeLogEvent> pending_commands_;
    std::mutex pending_mutex_;

    std::ofstream log_file_;
    std::mutex file_mutex_;
};

uint64_t getMonotonicTimeNs();

}  // namespace ethercat_joint

#endif  // RUNTIME_LOGGER_HPP
