#include "ethercat_joint/master/igh/igh_master_runtime.hpp"

#include "ethercat_joint/servo/ethercat_servo.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>

namespace ethercat_joint
{
namespace
{

uint64_t monoNowNs()
{
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull +
         static_cast<uint64_t>(ts.tv_nsec);
}

void nsToTimespec(uint64_t ns, timespec & ts)
{
  ts.tv_sec = static_cast<time_t>(ns / 1000000000ull);
  ts.tv_nsec = static_cast<long>(ns % 1000000000ull);
}

void updateAtomicMax(std::atomic<uint64_t> & target, uint64_t value)
{
  uint64_t current = target.load(std::memory_order_relaxed);
  while (value > current &&
         !target.compare_exchange_weak(
           current, value, std::memory_order_relaxed, std::memory_order_relaxed))
  {
  }
}

}  // namespace

IghMasterRuntime & IghMasterRuntime::instance()
{
  static IghMasterRuntime runtime;
  return runtime;
}

bool IghMasterRuntime::configure(const IghMasterConfig & config)
{
  if (job_running_.load(std::memory_order_acquire)) {
    return false;
  }
  config_ = config;
  wkc_tracker_ = AnomalyTracker(config_.wkc_anomaly_policy);
  deadline_tracker_ = AnomalyTracker(config_.deadline_anomaly_policy);
  dc_tracker_ = AnomalyTracker(config_.dc_anomaly_policy);
  dc_warmup_remaining_ = config_.dc_monitor_warmup_cycles;
  dc_status_valid_.store(false, std::memory_order_relaxed);
  dc_in_sync_.store(false, std::memory_order_relaxed);
  dc_deviation_ns_.store(0, std::memory_order_relaxed);
  max_dc_deviation_ns_.store(0, std::memory_order_relaxed);
  dc_out_of_sync_count_.store(0, std::memory_order_relaxed);
  dc_out_of_sync_consecutive_.store(0, std::memory_order_relaxed);
  dc_out_of_sync_window_.store(0, std::memory_order_relaxed);
  healthy_dwell_.reset(config_.healthy_dwell_cycles);
  healthy_dwell_.allowInitialEnable();
  motion_reenable_allowed_.store(true, std::memory_order_release);
  return applyMemoryLock();
}

bool IghMasterRuntime::applyMemoryLock()
{
  if (!config_.lock_memory) {
    std::cerr << "[IgH] IGH_LOCK_MEMORY=0: skipping mlockall" << std::endl;
    return true;
  }
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    const int err = errno;
    std::cerr << "[IgH] mlockall failed: " << std::strerror(err) << std::endl;
    if (config_.require_realtime) {
      return false;
    }
    std::cerr << "[IgH] IGH_REQUIRE_REALTIME=0: continuing without mlockall" << std::endl;
    return true;
  }
  std::cerr << "[IgH] mlockall(MCL_CURRENT|MCL_FUTURE) ok" << std::endl;
  return true;
}

bool IghMasterRuntime::applyThreadRealtime(int fifo_priority, std::atomic<bool> * rt_ok_out)
{
  bool ok = true;
  if (config_.cpu_affinity >= 0) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(config_.cpu_affinity, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
      std::cerr << "[IgH] pthread_setaffinity_np(cpu=" << config_.cpu_affinity
                << ") failed: " << std::strerror(errno) << std::endl;
      ok = false;
    }
  }

  sched_param scheduling{};
  scheduling.sched_priority = fifo_priority;
  const int sched_rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &scheduling);
  if (sched_rc != 0) {
    std::cerr << "[IgH] pthread_setschedparam(SCHED_FIFO," << fifo_priority
              << ") failed: " << std::strerror(sched_rc) << std::endl;
    ok = false;
  }

  if (rt_ok_out) {
    rt_ok_out->store(ok, std::memory_order_release);
  }
  if (!ok && config_.require_realtime) {
    std::cerr << "[IgH] IGH_REQUIRE_REALTIME=1: RT setup failed"
              << " (set IGH_REQUIRE_REALTIME=0 or fix LimitRTPRIO/affinity)"
              << std::endl;
    return false;
  }
  if (!ok) {
    std::cerr << "[IgH] IGH_REQUIRE_REALTIME=0: continuing without full RT" << std::endl;
  }
  return true;
}

void IghMasterRuntime::latchCommFault()
{
  comm_fault_.store(true, std::memory_order_release);
  safe_output_required_.store(true, std::memory_order_release);
  motion_reenable_allowed_.store(false, std::memory_order_release);
  pending_dwell_fault_.store(true, std::memory_order_release);
  healthy_dwell_.onFault();
}

void IghMasterRuntime::raiseCommFault()
{
  latchCommFault();
}

void IghMasterRuntime::clearCommFault()
{
  comm_fault_.store(false, std::memory_order_release);
  safe_output_required_.store(false, std::memory_order_release);
  pending_dwell_fault_.store(false, std::memory_order_release);
  wkc_tracker_.reset();
  deadline_tracker_.reset();
  dc_tracker_.reset();
  healthy_dwell_.requestReset();
  motion_reenable_allowed_.store(false, std::memory_order_release);
}

void IghMasterRuntime::syncMotionReenableFlag()
{
  motion_reenable_allowed_.store(healthy_dwell_.allowEnable(), std::memory_order_release);
}

IghJobCycleDiag IghMasterRuntime::jobCycleDiag() const
{
  IghJobCycleDiag d{};
  d.period_ns = static_cast<uint64_t>(std::max(1u, config_.bus_cycle_us)) * 1000ULL;
  d.lateness_ns = diag_lateness_ns_.load(std::memory_order_relaxed);
  d.execution_ns = diag_execution_ns_.load(std::memory_order_relaxed);
  d.max_lateness_ns = diag_max_lateness_ns_.load(std::memory_order_relaxed);
  d.max_execution_ns = diag_max_execution_ns_.load(std::memory_order_relaxed);
  d.deadline_miss_count = diag_deadline_miss_.load(std::memory_order_relaxed);
  d.skipped_slots = skipped_slots_.load(std::memory_order_relaxed);
  d.deadline_met = diag_deadline_met_.load(std::memory_order_relaxed);
  d.last_rx_ok = last_rx_ok_.load(std::memory_order_relaxed);
  d.comm_fault = comm_fault_.load(std::memory_order_relaxed);
  d.safe_output_required = safe_output_required_.load(std::memory_order_relaxed);
  d.dc_status_valid = dc_status_valid_.load(std::memory_order_relaxed);
  d.dc_in_sync = dc_in_sync_.load(std::memory_order_relaxed);
  d.dc_deviation_ns = dc_deviation_ns_.load(std::memory_order_relaxed);
  d.max_dc_deviation_ns = max_dc_deviation_ns_.load(std::memory_order_relaxed);
  d.dc_out_of_sync_count = dc_out_of_sync_count_.load(std::memory_order_relaxed);
  d.dc_out_of_sync_consecutive = dc_out_of_sync_consecutive_.load(std::memory_order_relaxed);
  d.dc_out_of_sync_window = dc_out_of_sync_window_.load(std::memory_order_relaxed);
  return d;
}

void IghMasterRuntime::sampleDcMonitor(EtherCATServo * servo)
{
  const bool valid = servo ? servo->isDcStatusValid() : false;
  const int32_t diff = servo ? servo->lastDcDiffNs() : 0;
  const DcStatusSample sample = makeDcStatusSample(
    valid, diff, static_cast<int32_t>(config_.dc_sync_threshold_ns));

  dc_status_valid_.store(sample.status_valid, std::memory_order_relaxed);
  dc_in_sync_.store(sample.in_sync, std::memory_order_relaxed);
  dc_deviation_ns_.store(sample.deviation_ns, std::memory_order_relaxed);
  updateAtomicMax(max_dc_deviation_ns_, absDeviationNs(sample.deviation_ns));

  if (!operational_.load(std::memory_order_acquire)) {
    return;
  }

  if (dc_warmup_remaining_ > 0U) {
    --dc_warmup_remaining_;
    return;
  }

  if (comm_fault_.load(std::memory_order_acquire)) {
    return;
  }

  const bool anomaly = dcOutOfSyncAnomaly(sample);
  const AnomalyStats dc_stats = dc_tracker_.observe(anomaly);
  dc_out_of_sync_count_.store(dc_stats.total, std::memory_order_relaxed);
  dc_out_of_sync_consecutive_.store(dc_stats.consecutive, std::memory_order_relaxed);
  dc_out_of_sync_window_.store(dc_stats.window_count, std::memory_order_relaxed);
  if (dc_stats.stop_required) {
    latchCommFault();
  }
}

void IghMasterRuntime::timingThreadMain(IghMasterRuntime * self)
{
  if (!self->applyThreadRealtime(kTimingFifoPriority, &self->timing_rt_ok_)) {
    self->timing_running_.store(false, std::memory_order_release);
    return;
  }

  const uint64_t cycle_ns =
    static_cast<uint64_t>(std::max(1u, self->config_.bus_cycle_us)) * 1000ULL;
  uint64_t scheduled_wakeup_ns = monoNowNs() + cycle_ns;

  while (self->timing_running_.load(std::memory_order_acquire)) {
    timespec wake{};
    nsToTimespec(scheduled_wakeup_ns, wake);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake, nullptr);

    const uint64_t now_ns = monoNowNs();
    self->published_scheduled_wakeup_ns_.store(scheduled_wakeup_ns, std::memory_order_release);
    self->timing_tick_.fetch_add(1, std::memory_order_release);

    const uint64_t next = nextCycleDeadline(scheduled_wakeup_ns, now_ns, cycle_ns);
    const uint64_t skipped = skippedSlotsBetween(scheduled_wakeup_ns, next, cycle_ns);
    if (skipped > 0) {
      self->skipped_slots_.fetch_add(skipped, std::memory_order_relaxed);
    }
    scheduled_wakeup_ns = next;
  }
}

void IghMasterRuntime::jobThreadMain(IghMasterRuntime * self)
{
  // Timing 线程递增 timing_tick_ → 本 Job 醒来 → runJobCycle(RX/DC sync/TX) →
  // WKC/deadline/DC AnomalyTracker（仅 operational_）→ healthy dwell。
  // Healthy 组成：rx_ok ∧ deadline_met ∧ dc_ok ∧ !comm_fault。
  if (!self->applyThreadRealtime(kJobFifoPriority, &self->job_rt_ok_)) {
    self->job_running_.store(false, std::memory_order_release);
    return;
  }

  const uint64_t cycle_ns =
    static_cast<uint64_t>(std::max(1u, self->config_.bus_cycle_us)) * 1000ULL;
  EtherCATServo * servo = self->servo_;
  if (!servo) {
    self->job_running_.store(false, std::memory_order_release);
    return;
  }

  while (self->job_running_.load(std::memory_order_acquire)) {
    // Wait for Timing tick (bounded spin + short sleep to avoid busy-burn)
    uint64_t tick = self->timing_tick_.load(std::memory_order_acquire);
    while (tick == self->job_seen_tick_ &&
           self->job_running_.load(std::memory_order_acquire))
    {
      timespec pause{0, 50000};  // 50 us
      nanosleep(&pause, nullptr);
      tick = self->timing_tick_.load(std::memory_order_acquire);
    }
    if (!self->job_running_.load(std::memory_order_acquire)) {
      break;
    }
    self->job_seen_tick_ = tick;

    const uint64_t scheduled =
      self->published_scheduled_wakeup_ns_.load(std::memory_order_acquire);
    const uint64_t actual_wakeup = monoNowNs();

    if (self->pending_dwell_fault_.exchange(false, std::memory_order_acq_rel)) {
      self->healthy_dwell_.onFault();
    }

    const bool armed = self->operational_.load(std::memory_order_acquire);
    const bool rx_ok = servo->runJobCycle(self->safeOutputRequired());
    self->last_rx_ok_.store(rx_ok, std::memory_order_relaxed);

    if (armed && !self->commFault()) {
      const auto wkc = self->wkc_tracker_.observe(!rx_ok);
      if (wkc.stop_required) {
        self->latchCommFault();
      }
    }

    const uint64_t cycle_end = monoNowNs();
    self->timing_stats_ = observeCycleTiming(
      self->timing_stats_, scheduled, actual_wakeup, cycle_end, cycle_ns);
    self->diag_lateness_ns_.store(self->timing_stats_.current_lateness_ns, std::memory_order_relaxed);
    self->diag_execution_ns_.store(self->timing_stats_.current_execution_ns, std::memory_order_relaxed);
    self->diag_max_lateness_ns_.store(self->timing_stats_.max_lateness_ns, std::memory_order_relaxed);
    self->diag_max_execution_ns_.store(self->timing_stats_.max_execution_ns, std::memory_order_relaxed);
    self->diag_deadline_miss_.store(self->timing_stats_.deadline_miss_count, std::memory_order_relaxed);
    self->diag_deadline_met_.store(self->timing_stats_.deadline_met, std::memory_order_relaxed);

    if (armed && !self->commFault()) {
      const auto dl = self->deadline_tracker_.observe(!self->timing_stats_.deadline_met);
      if (dl.stop_required) {
        self->latchCommFault();
      }
    }

    self->sampleDcMonitor(servo);

    const bool dc_ok =
      !self->dc_status_valid_.load(std::memory_order_relaxed) ||
      self->dc_in_sync_.load(std::memory_order_relaxed);
    const bool healthy =
      rx_ok && self->timing_stats_.deadline_met && dc_ok && !self->commFault();
    self->healthy_dwell_.observe(healthy);
    self->syncMotionReenableFlag();

    if (self->config_.debug_log) {
      static uint64_t log_div = 0;
      if ((++log_div % 250) == 0) {
        const auto d = self->jobCycleDiag();
        std::cerr << "[IgH] JobDiag late=" << d.lateness_ns
                  << " exec=" << d.execution_ns
                  << " miss=" << d.deadline_miss_count
                  << " skip=" << d.skipped_slots
                  << " rx=" << d.last_rx_ok
                  << " fault=" << d.comm_fault
                  << " dc_valid/sync=" << (d.dc_status_valid ? 1 : 0) << "/"
                  << (d.dc_in_sync ? 1 : 0)
                  << " dc_dev=" << d.dc_deviation_ns << std::endl;
      }
    }
  }
}

bool IghMasterRuntime::start(EtherCATServo * servo)
{
  if (!servo) {
    return false;
  }
  if (job_running_.load(std::memory_order_acquire)) {
    return true;
  }

  if (!configure(IghMasterConfig::fromEnvironment())) {
    std::cerr << "[IgH] configure/mlock failed (fail-closed)" << std::endl;
    return false;
  }

  servo_ = servo;
  operational_.store(true, std::memory_order_release);
  comm_fault_.store(false, std::memory_order_release);
  safe_output_required_.store(false, std::memory_order_release);
  healthy_dwell_.allowInitialEnable();
  motion_reenable_allowed_.store(true, std::memory_order_release);
  timing_tick_.store(0, std::memory_order_relaxed);
  job_seen_tick_ = 0;
  skipped_slots_.store(0, std::memory_order_relaxed);
  timing_stats_ = {};

  timing_rt_ok_.store(false, std::memory_order_relaxed);
  job_rt_ok_.store(false, std::memory_order_relaxed);
  timing_running_.store(true, std::memory_order_release);
  job_running_.store(true, std::memory_order_release);

  timing_thread_ = std::make_unique<std::thread>(timingThreadMain, this);
  job_thread_ = std::make_unique<std::thread>(jobThreadMain, this);

  // Brief settle so RT setup runs
  timespec settle{0, 20 * 1000 * 1000};
  nanosleep(&settle, nullptr);

  if (config_.require_realtime &&
      (!timing_rt_ok_.load(std::memory_order_acquire) ||
       !job_rt_ok_.load(std::memory_order_acquire)))
  {
    std::cerr << "[IgH] Timing/Job RT setup failed (fail-closed)" << std::endl;
    stop();
    return false;
  }

  std::cerr << "[IgH] hard-RT Job started bus_cycle_us=" << config_.bus_cycle_us
            << " affinity=" << config_.cpu_affinity << std::endl;
  return true;
}

void IghMasterRuntime::stop()
{
  timing_running_.store(false, std::memory_order_release);
  job_running_.store(false, std::memory_order_release);
  if (timing_thread_ && timing_thread_->joinable()) {
    timing_thread_->join();
  }
  if (job_thread_ && job_thread_->joinable()) {
    job_thread_->join();
  }
  timing_thread_.reset();
  job_thread_.reset();
  servo_ = nullptr;
  operational_.store(false, std::memory_order_release);
}

}  // namespace ethercat_joint
