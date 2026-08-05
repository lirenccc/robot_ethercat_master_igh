/**
 * @file igh_master_runtime.hpp
 * @brief IgH Timing + Job 硬实时周期所有者
 *
 * Master::cycle() 只灌 setpoint；PDO 交换仅在 Job 线程执行。
 */

#ifndef ETHERCAT_JOINT_MASTER_IGH_IGH_MASTER_RUNTIME_HPP
#define ETHERCAT_JOINT_MASTER_IGH_IGH_MASTER_RUNTIME_HPP

#include "ethercat_joint/master/igh/cycle_timing.hpp"
#include "ethercat_joint/master/igh/dc_monitor.hpp"
#include "ethercat_joint/master/igh/igh_master_config.hpp"
#include "ethercat_joint/util/anomaly_tracker.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>

namespace ethercat_joint
{

class EtherCATServo;

struct IghJobCycleDiag
{
  uint64_t period_ns{0};
  uint64_t lateness_ns{0};
  uint64_t execution_ns{0};
  uint64_t max_lateness_ns{0};
  uint64_t max_execution_ns{0};
  uint64_t deadline_miss_count{0};
  uint64_t skipped_slots{0};
  bool deadline_met{true};
  bool last_rx_ok{true};
  bool comm_fault{false};
  bool safe_output_required{false};
  bool dc_status_valid{false};
  bool dc_in_sync{false};
  int32_t dc_deviation_ns{0};
  uint64_t max_dc_deviation_ns{0};
  uint64_t dc_out_of_sync_count{0};
  uint32_t dc_out_of_sync_consecutive{0};
  uint32_t dc_out_of_sync_window{0};
};

class IghMasterRuntime
{
public:
  static IghMasterRuntime & instance();

  IghMasterRuntime(const IghMasterRuntime &) = delete;
  IghMasterRuntime & operator=(const IghMasterRuntime &) = delete;

  bool configure(const IghMasterConfig & config);
  bool start(EtherCATServo * servo);
  void stop();

  bool isJobThreadRunning() const
  {
    return job_running_.load(std::memory_order_acquire);
  }

  uint32_t busCycleUs() const { return config_.bus_cycle_us; }

  bool commFault() const { return comm_fault_.load(std::memory_order_acquire); }
  bool safeOutputRequired() const
  {
    return safe_output_required_.load(std::memory_order_acquire);
  }
  bool motionReenableAllowed() const
  {
    return motion_reenable_allowed_.load(std::memory_order_acquire);
  }

  void raiseCommFault();
  void clearCommFault();
  void requestSafeOutput();
  bool releaseSafeOutput();

  void setOperational(bool operational)
  {
    operational_.store(operational, std::memory_order_release);
  }

  IghJobCycleDiag jobCycleDiag() const;

private:
  IghMasterRuntime() = default;

  bool applyMemoryLock();
  bool applyThreadRealtime(int fifo_priority, std::atomic<bool> * rt_ok_out);
  void latchCommFault();
  void syncMotionReenableFlag();
  void sampleDcMonitor(EtherCATServo * servo);

  static void timingThreadMain(IghMasterRuntime * self);
  static void jobThreadMain(IghMasterRuntime * self);

  static constexpr int kTimingFifoPriority = 98;
  static constexpr int kJobFifoPriority = 99;  // 单 Job 线程时给最高 FIFO（无 Timing 抢核）

  IghMasterConfig config_{};
  EtherCATServo * servo_{nullptr};

  std::atomic<bool> timing_running_{false};
  std::atomic<bool> job_running_{false};
  std::atomic<bool> timing_rt_ok_{false};
  std::atomic<bool> job_rt_ok_{false};
  std::atomic<bool> operational_{false};
  std::atomic<bool> comm_fault_{false};
  std::atomic<bool> safe_output_required_{false};
  std::atomic<bool> motion_reenable_allowed_{true};
  std::atomic<bool> pending_dwell_fault_{false};

  std::atomic<uint64_t> published_scheduled_wakeup_ns_{0};
  std::atomic<uint64_t> skipped_slots_{0};
  std::atomic<bool> last_rx_ok_{true};
  mutable std::atomic<uint64_t> diag_lateness_ns_{0};
  mutable std::atomic<uint64_t> diag_execution_ns_{0};
  mutable std::atomic<uint64_t> diag_max_lateness_ns_{0};
  mutable std::atomic<uint64_t> diag_max_execution_ns_{0};
  mutable std::atomic<uint64_t> diag_deadline_miss_{0};
  mutable std::atomic<bool> diag_deadline_met_{true};

  std::unique_ptr<std::thread> timing_thread_;
  std::unique_ptr<std::thread> job_thread_;

  // Timing → Job 唤醒（eventfd-like via atomic + busy wait with nanosleep fallback）
  std::atomic<uint64_t> timing_tick_{0};
  uint64_t job_seen_tick_{0};

  AnomalyTracker wkc_tracker_;
  AnomalyTracker deadline_tracker_;
  AnomalyTracker dc_tracker_;
  uint32_t dc_warmup_remaining_{0};
  std::atomic<bool> dc_status_valid_{false};
  std::atomic<bool> dc_in_sync_{false};
  std::atomic<int32_t> dc_deviation_ns_{0};
  std::atomic<uint64_t> max_dc_deviation_ns_{0};
  std::atomic<uint64_t> dc_out_of_sync_count_{0};
  std::atomic<uint32_t> dc_out_of_sync_consecutive_{0};
  std::atomic<uint32_t> dc_out_of_sync_window_{0};
  HealthyDwell healthy_dwell_;
  CycleTimingStats timing_stats_{};
};

}  // namespace ethercat_joint

#endif
