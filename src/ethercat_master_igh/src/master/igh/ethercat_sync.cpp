/**
 * @file ethercat_sync.cpp
 * @brief EtherCAT 分布式时钟同步实现
 */

#include "ethercat_joint/master/igh/ethercat_sync.hpp"

#define CLOCK_TO_USE CLOCK_MONOTONIC
#define NSEC_PER_SEC 1000000000ULL

namespace ethercat_joint {

int EtherCATSync::readReferenceClockTimeQuiet(uint32_t* time_low)
{
    if (!master_ || !time_low) {
        return -1;
    }
    // 对齐天机：直接读回。禁止在 RT 热路径 dup2/open/close（实测可把 2ms
    // 周期拖到 100ms+，WKC 永不完整 → areAllSlavesInOP 假超时）。
    // libethercat 失败时可能打 stderr；失败退避后尝试间隔拉长，刷屏可控。
    return ecrt_master_reference_clock_time(master_, time_low);
}
 
 EtherCATSync::EtherCATSync()
     : master_(nullptr), cycle_time_ns_(0),
       dc_start_time_ns_(0), dc_time_ns_(0), dc_started_(false),
       dc_diff_ns_(0), prev_dc_diff_ns_(0),
       dc_diff_total_ns_(0), dc_delta_total_ns_(0),
       dc_filter_idx_(0), dc_adjust_ns_(0), sys_time_base_(0)
 {
 }
 
 EtherCATSync::~EtherCATSync() {
 }
 
 void EtherCATSync::initialize(ec_master_t* master, uint32_t cycle_time_ns)
 {
     master_ = master;
     cycle_time_ns_ = cycle_time_ns;
 
     // 初始化时间
     dc_start_time_ns_ = getMonotonicTime();
     dc_time_ns_ = dc_start_time_ns_;
     sys_time_base_ = 0;
     
     dc_started_ = false;
     dc_diff_ns_ = 0;
     prev_dc_diff_ns_ = 0;

    warmup_cycles_remaining_ = kRefClockWarmupCycles;
    consecutive_ref_successes_ = 0;
    consecutive_ref_failures_ = 0;
    ref_clock_process_cycle_ = 0;
    ref_clock_fail_streak_ = 0;
    ref_clock_backoff_ = false;
    last_good_dc_diff_ns_ = 0;
    have_good_dc_diff_ = false;
    ref_clock_read_disabled_ = false;
     ref_clock_disable_logged_ = false;
     
     // 设置初始的应用时间
     ecrt_master_application_time(master_, dc_start_time_ns_);
     
     std::cout << "EtherCAT Sync Initialized. Cycle: " << cycle_time_ns_ << " ns" << std::endl;
 }
 
 uint64_t EtherCATSync::getMonotonicTime()
 {
     struct timespec ts;
     clock_gettime(CLOCK_TO_USE, &ts);
     return (uint64_t)ts.tv_sec * NSEC_PER_SEC + ts.tv_nsec;
 }
 
struct timespec EtherCATSync::getSleepSpec(uint64_t wakeup_time_ns) const
{
    struct timespec ts;
    // 关键：将逻辑唤醒时间加上漂移补偿基准，得到实际的系统睡眠时间
    uint64_t target_system_time = wakeup_time_ns + sys_time_base_;
    
    ts.tv_sec = target_system_time / NSEC_PER_SEC;
    ts.tv_nsec = target_system_time % NSEC_PER_SEC;
    return ts;
}
 
int32_t EtherCATSync::process(uint64_t app_time_ns)
{
    if (!master_) {
        return 0;
    }

    // 当前周期的逻辑应用时间。该值必须与
    // ecrt_master_application_time() 使用的时间保持一致。
    dc_time_ns_ = app_time_ns;

    bool have_reference_time = false;
    uint32_t reference_time_low = 0;

    if (warmup_cycles_remaining_ > 0) {
        --warmup_cycles_remaining_;
    }

    /*
     * ecrt_master_reference_clock_time() 读取的是上一周期已经发送、
     * 并在本周期 ecrt_master_receive() 中收到的参考时钟数据报结果。
     *
     * 对齐天机参考实现：无 reference_clock_ready 门控；PREOP 阶段读回可能失败，
     * 由退避逻辑处理。DC 同步数据报始终排入，避免 SAFEOP/OP ↔ DC 循环依赖。
     */
    const bool may_read_ref_clock =
        !ref_clock_read_disabled_ && warmup_cycles_remaining_ == 0;

    if (may_read_ref_clock) {
        // 参考时钟读回失败退避（对齐参考实现）：失败越多尝试间隔越长，
        // 不永久禁用；成功一次即记录 last_good，失败时保持补偿值避免突变。
        ++ref_clock_process_cycle_;
        bool attempt_ref_read = true;
        if (ref_clock_backoff_) {
            uint64_t interval = kRefClockBackoffEveryCycles;
            if (ref_clock_fail_streak_ >= kRefClockFailEnterBackoff * 4) {
                interval = kRefClockBackoffMaxCycles;
            } else if (ref_clock_fail_streak_ >= kRefClockFailEnterBackoff * 2) {
                interval = kRefClockBackoffEveryCycles * 5;
            }
            attempt_ref_read = (ref_clock_process_cycle_ % interval == 0);
        }
        if (attempt_ref_read &&
            readReferenceClockTimeQuiet(&reference_time_low) == 0) {
            const uint32_t app_time_low = static_cast<uint32_t>(dc_time_ns_);
            dc_diff_ns_ = static_cast<int32_t>(app_time_low - reference_time_low);
            have_reference_time = true;
            consecutive_ref_failures_ = 0;
            ref_clock_fail_streak_ = 0;
            ref_clock_backoff_ = false;
            last_good_dc_diff_ns_ = dc_diff_ns_;
            have_good_dc_diff_ = true;
            ++consecutive_ref_successes_;
        } else if (attempt_ref_read) {
            // 本拍尝试读回但失败：保持上次有效差值，避免 DC 补偿突变
            dc_diff_ns_ = have_good_dc_diff_ ? last_good_dc_diff_ns_ : 0;
            consecutive_ref_successes_ = 0;
            ++consecutive_ref_failures_;
            ++ref_clock_fail_streak_;
            if (ref_clock_fail_streak_ == kRefClockFailEnterBackoff) {
                ref_clock_backoff_ = true;
                std::cerr << "DC reference clock read failed ("
                          << ref_clock_fail_streak_
                          << "); backoff engaged, sync frames continue"
                          << std::endl;
            }
        } else {
            // 退避间隔内不读：保持 last_good
            dc_diff_ns_ = have_good_dc_diff_ ? last_good_dc_diff_ns_ : 0;
        }
    } else if (!ref_clock_read_disabled_) {
        dc_diff_ns_ = 0;
    }

    /*
     * 为下一次 ecrt_master_send() 排入 DC 同步数据报。
     * 即使从站当前仍在 PREOP，也必须持续发送这些数据报，以便主站
     * 状态机能够顺利推进到 SAFEOP/OP。
     */
    ecrt_master_sync_reference_clock(master_);
    ecrt_master_sync_slave_clocks(master_);

    // 需要连续多次成功读回后再启动 PLL，避免单次误读触发 "DC Sync Started"
    if (!have_reference_time ||
        consecutive_ref_successes_ < kRefClockRequiredSuccesses) {
        return 0;
    }

    // 执行漂移补偿算法（Update Master Clock / PLL）。
    const int32_t delta = dc_diff_ns_ - prev_dc_diff_ns_;
    prev_dc_diff_ns_ = dc_diff_ns_;

    // 将误差归一化到 [-cycle/2, cycle/2)；避免 32 位时间回绕影响。
    const int32_t cycle = static_cast<int32_t>(cycle_time_ns_);
    dc_diff_ns_ =
        ((dc_diff_ns_ + cycle / 2) % cycle + cycle) % cycle - cycle / 2;

    if (dc_started_) {
        dc_diff_total_ns_ += dc_diff_ns_;
        dc_delta_total_ns_ += delta;
        ++dc_filter_idx_;

        if (dc_filter_idx_ >= DC_FILTER_CNT) {
            const int64_t delta_avg =
                (dc_delta_total_ns_ + DC_FILTER_CNT / 2) / DC_FILTER_CNT;
            const int64_t error_compensation =
                sign(dc_diff_total_ns_ / DC_FILTER_CNT);

            int64_t new_adjust = delta_avg + error_compensation;

            // 单次调整限制为 ±1 us，防止时基突变。
            constexpr int64_t kAdjustLimitNs = 1000;
            if (new_adjust < -kAdjustLimitNs) {
                new_adjust = -kAdjustLimitNs;
            } else if (new_adjust > kAdjustLimitNs) {
                new_adjust = kAdjustLimitNs;
            }

            // 平滑调整量，降低周期抖动。
            constexpr double kSmoothingFactor = 0.5;
            dc_adjust_ns_ = static_cast<int64_t>(
                dc_adjust_ns_ * (1.0 - kSmoothingFactor) +
                new_adjust * kSmoothingFactor);

            dc_diff_total_ns_ = 0;
            dc_delta_total_ns_ = 0;
            dc_filter_idx_ = 0;
        }

        sys_time_base_ += dc_adjust_ns_;
    } else {
        dc_started_ = true;
        sys_time_base_ = 0;
        std::cout << "DC Sync Started. Initial Diff: "
                  << dc_diff_ns_ << " ns" << std::endl;
    }

    return dc_diff_ns_;
 }

 uint64_t EtherCATSync::getApplicationTime()
 {
     return getMonotonicTime() - sys_time_base_;
 }
 
 } // namespace ethercat_joint
