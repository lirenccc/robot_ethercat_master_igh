/**
 * @file ethercat_sync.hpp
 * @brief EtherCAT 分布式时钟同步与漂移补偿类
 */

 #ifndef ETHERCAT_SYNC_HPP
 #define ETHERCAT_SYNC_HPP
 
 #include <time.h>
 #include <stdint.h>
 #include <cmath>
 #include <iostream>
 
 extern "C" {
 #include "ecrt.h"
 }
 
 namespace ethercat_joint {
 
 class EtherCATSync {
 public:
     EtherCATSync();
     ~EtherCATSync();
 
     /**
      * @brief 初始化同步模块
      * @param master EtherCAT主站指针
      * @param cycle_time_ns 周期时间（纳秒）
      */
     void initialize(ec_master_t* master, uint32_t cycle_time_ns);
 
    /**
     * @brief 执行DC同步核心逻辑 (对应 dc.c 中的 sync_distributed_clocks + update_master_clock)
     * @param app_time_ns 当前应用时间（逻辑时间），用于计算时间差
     * @return 当前周期的漂移值（ns）
     */
    int32_t process(uint64_t app_time_ns);
 
     /**
      * @brief 将逻辑应用时间（Wakeup Time）转换为包含漂移补偿的系统绝对时间
      * 用于 clock_nanosleep
      */
     struct timespec getSleepSpec(uint64_t wakeup_time_ns);
 
     /**
      * @brief 获取当前的单调系统时间（纳秒）
      */
     uint64_t getMonotonicTime();
 
     /**
      * @brief 获取主站应用时间（Master Application Time）
      * 实际上是 master_time - system_base
      */
     uint64_t getApplicationTime();
 
     /**
      * @brief 检查DC是否已同步/启动
      */
     bool isSynced() const { return dc_started_; }

     /**
      * @brief 允许读取参考时钟数据报结果并启动 PLL。
      * @note 此标志不会禁止 DC 同步数据报发送；PREOP 阶段仍会持续排入同步报文。
      */
     void setReferenceClockReady(bool ready) { reference_clock_ready_ = ready; }

     /** PLL 读回是否已因持续失败而停用（DC 同步帧仍会发送） */
     bool isReferenceClockReadDisabled() const {
         return ref_clock_read_disabled_;
     }

     /** 当前周期 DC 偏差（ns），由 process() 更新 */
     int32_t lastDcDiffNs() const { return dc_diff_ns_; }

     /** 参考时钟可读且 PLL 已启动（未停用读回） */
     bool isDcPllActive() const {
         return dc_started_ && reference_clock_ready_ && !ref_clock_read_disabled_;
     }
 
 private:
     /** 调用 ecrt_master_reference_clock_time，抑制 libethercat 的 stderr 刷屏 */
     int readReferenceClockTimeQuiet(uint32_t* time_low);

     ec_master_t* master_;
     uint32_t cycle_time_ns_;
 
     // === DC 算法状态变量 (源自 dc.c) ===
     const int32_t DC_FILTER_CNT = 1024;
     
     uint64_t dc_start_time_ns_;
     uint64_t dc_time_ns_;      // 下一个周期的预期应用时间
     bool dc_started_;
     
     int32_t dc_diff_ns_;       // 当前误差
     int32_t prev_dc_diff_ns_;  // 上一次误差
     
     int64_t dc_diff_total_ns_; // 误差累积
     int64_t dc_delta_total_ns_;// 变化量累积
     int32_t dc_filter_idx_;
     int64_t dc_adjust_ns_;     // 计算出的调整量
 
     int64_t sys_time_base_;    // 系统时间基准偏移量 (核心变量)

     bool reference_clock_ready_ = false;

     // 参考时钟读回：预热 / 连续成功 / 失败退避
     static constexpr int kRefClockWarmupCycles = 100;
     static constexpr int kRefClockRequiredSuccesses = 3;
     static constexpr int kRefClockMaxFailures = 30;
     int warmup_cycles_remaining_ = kRefClockWarmupCycles;
     int consecutive_ref_successes_ = 0;
     int consecutive_ref_failures_ = 0;
     bool ref_clock_read_disabled_ = false;
     bool ref_clock_disable_logged_ = false;
 
     // 辅助函数：符号函数
     template <typename T> 
     int sign(T val) {
         return (val > 0) - (val < 0);
     }
 };
 
 } // namespace ethercat_joint
 
 #endif // ETHERCAT_SYNC_HPP