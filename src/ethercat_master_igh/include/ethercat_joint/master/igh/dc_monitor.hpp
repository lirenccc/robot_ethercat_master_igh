/**
 * @file dc_monitor.hpp
 * @brief IgH DC 偏差样本解读（纯头文件）
 *
 * status_valid：参考时钟可读且 PLL 已启动；in_sync：|deviation| 未超阈值。
 * 仅在 status_valid 时把 !in_sync 计为异常，避免启动预热期误闩锁。
 */

#ifndef ETHERCAT_JOINT_MASTER_IGH_DC_MONITOR_HPP
#define ETHERCAT_JOINT_MASTER_IGH_DC_MONITOR_HPP

#include <cstdint>
#include <cstdlib>

namespace ethercat_joint {

struct DcStatusSample {
    bool status_valid = false;
    bool in_sync = false;
    int32_t deviation_ns = 0;
};

inline DcStatusSample makeDcStatusSample(
    bool status_valid,
    int32_t deviation_ns,
    int32_t sync_threshold_ns) noexcept
{
    DcStatusSample s;
    s.status_valid = status_valid;
    s.deviation_ns = deviation_ns;
    const uint64_t abs_dev =
        static_cast<uint64_t>(std::llabs(static_cast<long long>(deviation_ns)));
    const uint64_t thr =
        static_cast<uint64_t>(sync_threshold_ns > 0 ? sync_threshold_ns : 0);
    s.in_sync = s.status_valid && abs_dev <= thr;
    return s;
}

inline bool dcOutOfSyncAnomaly(const DcStatusSample & sample) noexcept
{
    return sample.status_valid && !sample.in_sync;
}

inline uint64_t absDeviationNs(int32_t deviation_ns) noexcept
{
    return static_cast<uint64_t>(std::llabs(static_cast<long long>(deviation_ns)));
}

}  // namespace ethercat_joint

#endif
