/**
 * @file igh_master_config.hpp
 * @brief IgH 硬实时运行时配置（环境变量 IGH_*）
 */

#ifndef ETHERCAT_JOINT_MASTER_IGH_IGH_MASTER_CONFIG_HPP
#define ETHERCAT_JOINT_MASTER_IGH_IGH_MASTER_CONFIG_HPP

#include "ethercat_joint/util/anomaly_tracker.hpp"

#include <cstdint>
#include <cstdlib>

namespace ethercat_joint
{

namespace igh_detail
{

inline bool envFlagOr(const char * name, bool default_value)
{
  const char * v = std::getenv(name);
  if (v == nullptr || *v == '\0') {
    return default_value;
  }
  return *v != '0';
}

inline uint32_t envU32Or(const char * name, uint32_t default_value)
{
  const char * v = std::getenv(name);
  if (v == nullptr || *v == '\0') {
    return default_value;
  }
  const int parsed = std::atoi(v);
  return parsed > 0 ? static_cast<uint32_t>(parsed) : default_value;
}

/** 允许显式 `0`（如关闭 watchdog）；非法负数回退默认 */
inline uint32_t envU32AllowZeroOr(const char * name, uint32_t default_value)
{
  const char * v = std::getenv(name);
  if (v == nullptr || *v == '\0') {
    return default_value;
  }
  const int parsed = std::atoi(v);
  return parsed >= 0 ? static_cast<uint32_t>(parsed) : default_value;
}

}  // namespace igh_detail

struct IghMasterConfig
{
  /** 默认 1 ms，与 DC SYNC0 对齐；可用 IGH_BUS_CYCLE_US 覆盖 */
  uint32_t bus_cycle_us{1000};
  int cpu_affinity{5};
  bool debug_log{false};
  /** IGH_LOCK_MEMORY：启动尽早 mlockall；默认 true */
  bool lock_memory{true};
  /**
   * IGH_REQUIRE_REALTIME：mlock / FIFO / 亲和失败时拒绝启动；
   * 默认 true；开发机可显式设 0。
   */
  bool require_realtime{true};

  /**
   * 通信/超期/DC 异常策略（已实现 WKC + deadline + DC 独立监控）。
   * WKC：连续 5 或滑窗 16 内 ≥8 → latchCommFault。
   * Deadline：连续 10 或滑窗 32 内 ≥16 → latchCommFault。
   * DC：连续 5 或滑窗 16 内 ≥8 → latchCommFault（warmup 后武装）。
   * Healthy dwell：复位后连续 50 周期才允许再使能（@1ms ≈ 50 ms）。
   * 外部 CST 命令新鲜度：`cmd_watchdog_ms=0` 关闭；文件轨迹用 SetpointSource::JobInternal。
   */
  AnomalyPolicy wkc_anomaly_policy{5U, 16U, 8U};
  AnomalyPolicy deadline_anomaly_policy{10U, 32U, 16U};
  AnomalyPolicy dc_anomaly_policy{5U, 16U, 8U};
  uint32_t dc_monitor_warmup_cycles{100U};
  /** DC |deviation| 阈值（ns）；默认 0.5 ms */
  uint32_t dc_sync_threshold_ns{500000U};
  uint32_t healthy_dwell_cycles{50U};
  /** 外部 CST/CSV 命令新鲜度超时（ms）；0 关闭 */
  uint32_t cmd_watchdog_ms{250U};
  /** 是否对 CSV 外部速度同样武装 watchdog（默认关） */
  bool csv_cmd_watchdog{false};

  static IghMasterConfig fromEnvironment()
  {
    IghMasterConfig cfg;
    cfg.bus_cycle_us = igh_detail::envU32Or("IGH_BUS_CYCLE_US", cfg.bus_cycle_us);
    if (const char * cpu = std::getenv("IGH_CPU_AFFINITY")) {
      if (*cpu != '\0') {
        cfg.cpu_affinity = std::atoi(cpu);
      }
    }
    cfg.debug_log = igh_detail::envFlagOr("IGH_DEBUG_LOG", false);
    cfg.lock_memory = igh_detail::envFlagOr("IGH_LOCK_MEMORY", true);
    cfg.require_realtime = igh_detail::envFlagOr("IGH_REQUIRE_REALTIME", true);

    cfg.wkc_anomaly_policy.consecutive_stop =
      igh_detail::envU32Or("IGH_ANOMALY_WKC_CONSEC", cfg.wkc_anomaly_policy.consecutive_stop);
    cfg.wkc_anomaly_policy.window_size =
      igh_detail::envU32Or("IGH_ANOMALY_WKC_WINDOW", cfg.wkc_anomaly_policy.window_size);
    cfg.wkc_anomaly_policy.window_stop =
      igh_detail::envU32Or("IGH_ANOMALY_WKC_WINDOW_STOP", cfg.wkc_anomaly_policy.window_stop);

    cfg.deadline_anomaly_policy.consecutive_stop = igh_detail::envU32Or(
      "IGH_ANOMALY_DEADLINE_CONSEC", cfg.deadline_anomaly_policy.consecutive_stop);
    cfg.deadline_anomaly_policy.window_size = igh_detail::envU32Or(
      "IGH_ANOMALY_DEADLINE_WINDOW", cfg.deadline_anomaly_policy.window_size);
    cfg.deadline_anomaly_policy.window_stop = igh_detail::envU32Or(
      "IGH_ANOMALY_DEADLINE_WINDOW_STOP", cfg.deadline_anomaly_policy.window_stop);

    cfg.dc_anomaly_policy.consecutive_stop =
      igh_detail::envU32Or("IGH_ANOMALY_DC_CONSEC", cfg.dc_anomaly_policy.consecutive_stop);
    cfg.dc_anomaly_policy.window_size =
      igh_detail::envU32Or("IGH_ANOMALY_DC_WINDOW", cfg.dc_anomaly_policy.window_size);
    cfg.dc_anomaly_policy.window_stop =
      igh_detail::envU32Or("IGH_ANOMALY_DC_WINDOW_STOP", cfg.dc_anomaly_policy.window_stop);
    cfg.dc_monitor_warmup_cycles =
      igh_detail::envU32Or("IGH_DC_MONITOR_WARMUP", cfg.dc_monitor_warmup_cycles);
    cfg.dc_sync_threshold_ns =
      igh_detail::envU32Or("IGH_DC_SYNC_THRESHOLD_NS", cfg.dc_sync_threshold_ns);

    cfg.healthy_dwell_cycles =
      igh_detail::envU32Or("IGH_HEALTHY_DWELL_CYCLES", cfg.healthy_dwell_cycles);

    cfg.cmd_watchdog_ms =
      igh_detail::envU32AllowZeroOr("IGH_CMD_WATCHDOG_MS", cfg.cmd_watchdog_ms);
    cfg.csv_cmd_watchdog = igh_detail::envFlagOr("IGH_CSV_CMD_WATCHDOG", false);
    return cfg;
  }
};

}  // namespace ethercat_joint

#endif
