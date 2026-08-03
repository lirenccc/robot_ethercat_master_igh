/**
 * @file ethercat_servo.hpp
 * @brief EtherCAT 伺服驱动器类 - 支持 CSP/CSV/CST 模式
 */

#ifndef ETHERCAT_SERVO_IGH_HPP
#define ETHERCAT_SERVO_IGH_HPP

#include <iostream>
#include <vector>
#include <string>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <shared_mutex>
#include <atomic>
#include <deque>
#include <memory>
#include "ethercat_joint/master/igh/ethercat_sync.hpp"
#include "ethercat_joint/motor/motor_profile.hpp"
#include "ethercat_joint/diagnostics/runtime_logger.hpp"
#include "ethercat_joint/servo/cia402.hpp"
#include "ethercat_joint/util/command_freshness.hpp"
#include "ethercat_joint/util/seqlock.hpp"

extern "C" {
#include "ecrt.h"
}

namespace ethercat_joint {

struct IghJobCycleDiag;

/**
 * @brief 单个电机配置
 */
struct MotorConfig {
    uint16_t alias = 0;
    uint16_t position = 0;
    uint32_t vendor_id = 0;
    uint32_t product_code = 0;
    std::string name;
    std::string model_id;
    PdoLayout pdo_layout = PdoLayout::UNKNOWN;
};

/**
 * @brief 单个电机状态数据
 */
struct MotorStateData {
    uint8_t motor_id;
    OperationMode operation_mode;
    bool enabled;
    bool fault;
    uint16_t status_word;
    
    int32_t actual_position;
    int32_t target_position;
    int32_t actual_velocity;
    int32_t target_velocity;
    int16_t actual_torque;
    int32_t sensor_force_2020;  // SJD17 减速器输出端力矩传感器 (TxPDO 0x2020)；未映射时为 0
    int32_t motor_encoder_2021;  // SJD17 电机端（减速器输入端）编码器 (TxPDO 0x2021)；未映射时为 0
    int16_t target_torque;
    
    int8_t operation_mode_display;  // 0x6061: 操作模式显示寄存器
    
    uint32_t digital_inputs;
    uint32_t digital_outputs;
};

/**
 * @brief EtherCAT 伺服驱动器主类
 */
class EtherCATServo {
public:
    /**
     * @brief 构造函数
     * @param master_index 主站索引（通常为 0）
     */
    explicit EtherCATServo(unsigned int master_index = 0);
    
    /**
     * @brief 析构函数
     */
    ~EtherCATServo();
    
    /**
     * @brief 获取同步模块指针
     * 允许 Node 层访问时间转换函数
     */
     std::shared_ptr<EtherCATSync> getSyncHandler() const { return sync_handler_; }

     /**
      * @brief 执行同步逻辑
      * 在 cyclicTask 中调用，通常在 receiveData 之后
      */
     void processSync(uint64_t app_time_ns);
    
    /**
     * @brief 初始化 EtherCAT 主站和配置从站
     * @param motor_configs 电机配置列表
     * @return 成功返回 true
     */
    bool initialize(const std::vector<MotorConfig>& motor_configs);
    
    /**
     * @brief 激活主站（开始实时通信）
     * @return 成功返回 true
     */
    bool activate();
    
    /**
     * @brief 停用主站（停止实时通信 / Job）
     */
    void deactivate();

    /**
     * @brief Job 线程一拍：RX → DC sync → (safe|TX)。返回本拍 domain WC 是否完整。
     */
    bool runJobCycle(bool force_safe_output);

    bool isJobThreadRunning() const;
    uint32_t busCycleUs() const;
    bool commFault() const;
    bool safeOutputRequired() const;
    bool motionReenableAllowed() const;
    void clearCommFault();
    void requestSafeOutput();
    bool releaseSafeOutput();
    /** Safety reset entry: same as clearCommFault (clear latch + healthy dwell). */
    void requestSafetyReset() { clearCommFault(); }
    IghJobCycleDiag jobCycleDiag() const;
    void applySafeProcessImageOutputs();

    /** 启动证据门是否通过；失败时 observation-only，禁止使能 */
    bool startupEvidencePassed() const { return startup_evidence_passed_; }

    /** 适配层抢锁失败时对已武装 CST/CSV 做本拍安全退化 */
    void applyCommandContentionFallback();

    int32_t lastDcDiffNs() const;
    bool isDcStatusValid() const;

    /** Job 线程每拍开头：外部命令新鲜度检查 */
    void checkExternalCommandFreshness();
    
    /**
     * @brief 检查主站是否已激活
     * @return 已激活返回 true
     */
    bool isActivated() const { return activated_; }
    
    /**
     * @brief 周期任务 - 接收数据
     */
    void receiveData();
    
    /**
     * @brief 周期任务 - 发送数据
     */
    void sendData();
    
    /**
     * @brief 设置应用时间（分布式时钟）
     * @param app_time_ns 应用时间（纳秒）
     */
    void setApplicationTime(uint64_t app_time_ns);
    
    /**
     * @brief 同步参考时钟到指定时间
     * @param time_ns 时间（纳秒）
     */
    void syncReferenceClock(uint64_t time_ns);
    
    /**
     * @brief 同步从站时钟
     */
    void syncSlaveClocks();
    
    /**
     * @brief 获取DC参考时钟时间（纳秒）
     * @return DC参考时钟时间（纳秒），如果未激活则返回0
     */
    uint64_t getReferenceClockTime() const;
    
    /**
     * @brief 对齐DC时间基准到系统时间
     * @param system_time_ns 系统时间（纳秒，64位）
     * 
     * @details
     * 当DC参考时钟不可用时，将DC时间的64位累积变量对齐到系统时间。
     * 这样当DC恢复时，其64位时间将从系统时间开始累积，保持时间连续性。
     */
    void alignDcTimeBase(uint64_t system_time_ns);
    
    /**
     * @brief 检查域状态
     */
    void checkDomainState();
    
    /**
     * @brief 检查主站状态
     */
    void checkMasterState();
    
    /**
     * @brief 设置电机使能/失能
     * @param motor_id 电机 ID (0xFF = 所有电机)
     * @param enable true=使能, false=失能
     * @return 成功返回 true
     */
    bool setEnable(uint8_t motor_id, bool enable);
    
    /**
     * @brief 显式 CiA402 Fault Reset（须已在 safe-output 且轴失能）
     * @param motor_id 轴索引，0xFF=全体故障轴
     */
    bool requestFaultReset(uint8_t motor_id) noexcept;

    /**
     * @brief 设置操作模式
     * @param motor_id 电机 ID (0xFF = 所有电机)
     * @param mode 操作模式
     * @return 成功返回 true
     */
    bool setOperationMode(uint8_t motor_id, OperationMode mode);

    OperationMode getOperationMode(uint8_t motor_id) const;
    
    /**
     * @brief 设置目标位置 (CSP 模式)
     * @param motor_id 电机 ID
     * @param position 目标位置
     * @param override_filter 是否覆盖滤波器（默认false）
     */
    void setTargetPosition(uint8_t motor_id, int32_t position, bool override_filter = false);
    
    /**
     * @brief 设置静止时输入位置（CSP运动结束时调用）
     * @param motor_id 电机ID
     * @param position 静止保持位置（将同时更新buffer所有元素）
     */
    void setIdleInputPosition(uint8_t motor_id, int32_t position);
    
    /**
     * @brief 设置目标速度 (CSV 模式)
     * @param motor_id 电机 ID
     * @param velocity 目标速度（脉冲/秒）
     * @param override_idle 是否覆盖idle速度（默认false）
     */
    void setTargetVelocity(uint8_t motor_id, int32_t velocity, bool override_idle = false,
        SetpointSource source = SetpointSource::External);
    
    /**
     * @brief 设置静止输入速度（CSV模式）
     * @param motor_id 电机ID
     * @param velocity 静止速度（通常为0）
     */
    void setIdleInputVelocity(uint8_t motor_id, int32_t velocity);
    
    /**
     * @brief 设置目标力矩 (CST 模式)
     * @param motor_id 电机 ID
     * @param torque 目标力矩
     */
    void setTargetTorque(uint8_t motor_id, int16_t torque,
        SetpointSource source = SetpointSource::External);
    
    /**
     * @brief 获取实际位置（从PDO）
     * @param motor_id 电机 ID
     * @return 实际位置（脉冲）
     */
    int32_t getPosition(uint8_t motor_id) const;
    
    /**
     * @brief 获取目标位置（从内部状态）
     * @param motor_id 电机 ID
     * @return 目标位置（脉冲）
     */
    int32_t getTargetPosition(uint8_t motor_id) const;
    
    /**
     * @brief 获取实际速度（从PDO）
     * @param motor_id 电机 ID
     * @return 实际速度（脉冲/秒）
     */
    int32_t getVelocity(uint8_t motor_id) const;
    
    /**
     * @brief 获取目标速度（从内部状态）
     * @param motor_id 电机 ID
     * @return 目标速度（脉冲/秒）
     */
    int32_t getTargetVelocity(uint8_t motor_id) const;
    
    /**
     * @brief 获取实际力矩（从PDO）
     * @param motor_id 电机 ID
     * @return 实际力矩
     */
    int16_t getTorque(uint8_t motor_id) const;

    /**
     * @brief 获取减速器输出端力矩传感器原始值（PDO 0x2020）
     * @param motor_id 电机 ID
     * @return SJD17 映射时为 raw；NH17 等未映射时恒为 0
     */
    int32_t getSensorForce2020(uint8_t motor_id) const;

    /**
     * @brief 获取电机端（减速器输入端）编码器原始值（PDO 0x2021）
     * @param motor_id 电机 ID
     * @return SJD17 映射时为 raw；NH17 等未映射时恒为 0
     */
    int32_t getMotorEncoder2021(uint8_t motor_id) const;
    
    /**
     * @brief 获取目标力矩（从 pending_commands 缓冲区）
     * @param motor_id 电机 ID
     * @return 目标力矩（千分比）
     */
    int16_t getTargetTorque(uint8_t motor_id) const;
    
    /**
     * @brief 设置CSP模式位置滤波器大小
     * @param size 滤波器大小 (1-11)，1表示无滤波，默认3
     * @return 成功返回 true
     */
    bool setPositionFilterSize(size_t size);
    
    /**
     * @brief 获取CSP模式位置滤波器大小
     * @return 当前滤波器大小
     */
    size_t getPositionFilterSize() const { return position_filter_size_; }
    
    /**
     * @brief 获取所有电机状态
     * @return 电机状态数据向量
     */
    std::vector<MotorStateData> getMotorStates() const;
    
    /**
     * @brief 在激活前通过SDO读取初始位置（避免发送0值）
     * @return 是否成功初始化
     * @details 在activate()之前调用，使用SDO读取每个电机的实际位置：
     *          - 每个电机读取5次，选择最合理的值（去除异常值）
     *          - 设置target_positions_、idle_input_positions_和buffer为读取值
     *          - 避免系统启动时发送0值导致电机移动
     */
    bool initializePositionsFromSDO();

    /**
     * @brief 激活前从 SDO 0x200E/0x2016 读取减速比与位置模式，更新 MotorKinematics
     * @details 新奇读 0x2016 推断内/外圈；SJD17 跳过 0x2016，固定使用 motor_profile 编码器分辨率。
     *          手册 0x200D 为电机/减速器端编码器当前值（只读），不可作分辨率
     */
    bool tryLoadKinematicsFromSdo();
    
    
    /**
     * @brief 获取电机数量
     */
    size_t getMotorCount() const { return motor_count_; }
    
    /**
     * @brief 检查是否已初始化
     */
    bool isInitialized() const { return initialized_; }
    
    /**
     * @brief 检查是否有电机处于活跃控制模式（CSP/CSV/CST）
     * @return 如果有任何电机处于活跃模式则返回true
     */
    bool hasActiveMotors() const;
    
    /**
     * @brief 检查所有从站的状态，报告任何处于 PREOP 或错误状态的从站
     * @return 如果有从站出现问题则返回false
     */
    bool checkSlaveStates() const;
    
    /**
     * @brief 检查所有从站（0到motor_count_-1）是否都在OP状态
     * @return 如果所有从站都在OP状态则返回true
     */
    bool areAllSlavesInOP() const;
    
    /**
     * @brief CIA402 状态机控制
     */
    CIA402State getState(uint16_t status_word) const;

    /**
     * @brief 绑定运行时诊断日志器（由 Node 层持有生命周期）
     */
    void setRuntimeLogger(RuntimeLogger* logger) { runtime_logger_ = logger; }

    /**
     * @brief 通过 SDO 0x603F 读取 CiA402 错误码
     */
    uint16_t readErrorCode(uint8_t motor_id) const;

private:
    /**
     * @brief 时钟同步处理器
     */
    std::shared_ptr<EtherCATSync> sync_handler_;
    /**
     * @brief 配置 PDO 映射（通过 SDO）
     */
    bool configurePDOMapping();
    
    /**
     * @brief 注册 PDO 条目
     */
    bool registerPDOEntries();

    bool verifyPdoEvidence(size_t motor_id) const;
    bool verifyInterpolationPeriodGate();
    bool runStartupEvidenceGate();
    void disarmAllCommandFreshness();
    
    /**
     * @brief 操作模式转换
     */
    std::string operationModeToString(OperationMode mode) const;
    OperationMode stringToOperationMode(const std::string& mode_str) const;

    // EtherCAT 对象
    unsigned int master_index_;
    ec_master_t* master_;
    ec_domain_t* domain_;
    uint8_t* domain_pd_;
    
    // 状态变量
    ec_master_state_t master_state_;
    ec_domain_state_t domain_state_;
    
    // 从站配置
    size_t motor_count_;
    std::vector<ec_slave_config_t*> slave_configs_;
    std::vector<MotorConfig> motor_configs_;
    
    // PDO 偏移量（每个电机）
    struct PDOOffsets {
        // RxPDO (主站 -> 从站)
        unsigned int control_word;
        unsigned int operation_mode;
        unsigned int padding_rx;           // 填充字节 0x5fff
        unsigned int target_position;
        unsigned int target_velocity;
        unsigned int target_torque;
        unsigned int digital_outputs;
        
        // TxPDO (从站 -> 主站)
        unsigned int status_word;
        unsigned int operation_mode_display;
        unsigned int padding_tx;           // 填充字节 0x5fff
        unsigned int actual_position;
        unsigned int actual_velocity;
        unsigned int actual_torque;
        unsigned int sensor_force_2020;  // 0x2020；未映射时为 0
        unsigned int motor_encoder_2021;  // 0x2021；未映射时为 0
        unsigned int digital_inputs;
        
        // ⭐ 网关相关 PDO 偏移量（CAN/CANFD/RS485）
        unsigned int can_tx_id;            // CAN发送ID (0x2000:01)
        unsigned int can_tx_length;       // CAN发送长度 (0x2000:02)
        unsigned int can_tx_data;          // CAN发送数据 (0x2000:03)
        unsigned int can_rx_id;            // CAN接收ID (0x2001:01)
        unsigned int can_rx_length;        // CAN接收长度 (0x2001:02)
        unsigned int can_rx_data;          // CAN接收数据 (0x2001:03)
        
        unsigned int canfd_tx_id;          // CANFD发送ID (0x2000:01，复用CAN)
        unsigned int canfd_tx_length;      // CANFD发送长度 (0x2000:02，复用CAN)
        unsigned int canfd_tx_data;        // CANFD发送数据 (0x2000:03，复用CAN)
        unsigned int canfd_rx_id;          // CANFD接收ID (0x2001:01，复用CAN)
        unsigned int canfd_rx_length;     // CANFD接收长度 (0x2001:02，复用CAN)
        unsigned int canfd_rx_data;        // CANFD接收数据 (0x2001:03，复用CAN)
        
        unsigned int rs485_1_rx_length;   // RS485_1接收长度 (0x2002:01)
        unsigned int rs485_1_rx_data;      // RS485_1接收数据 (0x2002:02)
        unsigned int rs485_1_tx_length;    // RS485_1发送长度 (0x2003:01)
        unsigned int rs485_1_tx_data;      // RS485_1发送数据 (0x2003:02)
        
        unsigned int rs485_2_rx_length;   // RS485_2接收长度 (0x2004:01)
        unsigned int rs485_2_rx_data;      // RS485_2接收数据 (0x2004:02)
        unsigned int rs485_2_tx_length;    // RS485_2发送长度 (0x2005:01)
        unsigned int rs485_2_tx_data;      // RS485_2发送数据 (0x2005:02)
    };
    std::vector<PDOOffsets> pdo_offsets_;
    
    /**
     * @brief 发送数据 - 通用部分（控制字、操作模式等）
     */
    void sendDataCommon(size_t motor_id, const PDOOffsets& offsets);
    
    /**
     * @brief 发送数据 - CSP模式特定处理
     */
    void sendDataCSP(size_t motor_id, const PDOOffsets& offsets);
    
    /**
     * @brief 发送数据 - CSV模式特定处理
     */
    void sendDataCSV(size_t motor_id, const PDOOffsets& offsets);
    
    /**
     * @brief 发送数据 - CST模式特定处理
     */
    void sendDataCST(size_t motor_id, const PDOOffsets& offsets);

    void fillRuntimeEventTargets(RuntimeLogEvent& ev, size_t motor_id) const;
    void maybeLogTargetSetpoint(size_t motor_id, uint16_t status_word,
                                uint16_t cia402_state, uint16_t control_word);
    
    // 运行时数据（使能命令跨 RT/ROS 线程，使用原子包装）
    std::vector<AtomicBool> enable_requested_;
    std::vector<AtomicBool> desired_enable_;      // 外部请求的目标使能状态
    std::vector<AtomicBool> enable_fsm_active_;   // 使能状态机是否运行中
    std::vector<AtomicU8> enable_fsm_step_;       // 当前步骤索引
    std::vector<AtomicU16> enable_fsm_wait_;      // 步骤等待计数（周期数）
    std::vector<std::atomic<uint16_t>> control_word_states_;  // 每个电机的控制字状态 (0x06, 0x07, 0x0F, 0x1F) - 原子类型
    std::vector<int> fault_reset_counter_;        // 故障复位计数器（用于故障复位后的状态恢复）
    std::vector<uint8_t> fault_reset_phase_;      // 故障复位相位: 0=idle, 1=sending 0x80, 2=waiting after 0x80, 3=sending 0x06
    std::vector<int> enable_monitor_counter_;     // 使能监控计数器（用于检测意外失能）
    std::vector<int> enable_monitor_loss_counter_; // 连续非 OP 计数（去抖）
    std::vector<int> fault_persist_counter_;       // 连续 Fault 计数（去抖后立刻复位）
    std::vector<OperationMode> current_modes_;
    std::vector<std::atomic<int32_t>> target_positions_;      // 目标位置 - 原子类型
    std::vector<std::atomic<int32_t>> target_velocities_;    // 目标速度 - 原子类型
    std::vector<std::atomic<int16_t>> target_torques_;      // 目标力矩 - 原子类型
    
    // ⭐ 待处理命令缓冲区（ROS Service -> RT循环）
    struct PendingCommand {
        std::atomic<int32_t> position;
        std::atomic<int32_t> velocity;
        std::atomic<int16_t> torque;
        std::atomic<bool> position_valid;
        std::atomic<bool> velocity_valid;
        std::atomic<bool> torque_valid;
        
        // 默认构造函数（原子类型会自动初始化为0/false）
        PendingCommand() : position(0), velocity(0), torque(0), 
                          position_valid(false), velocity_valid(false), torque_valid(false) {}
        
        // 注意：虽然包含原子类型，但结构体本身可以移动（移动时原子类型会使用默认构造）
        // 删除复制构造函数和赋值运算符（原子类型不可复制）
        PendingCommand(const PendingCommand&) = delete;
        PendingCommand& operator=(const PendingCommand&) = delete;
        // 允许移动构造和移动赋值（移动时原子类型会使用默认构造）
        PendingCommand(PendingCommand&&) noexcept = default;
        PendingCommand& operator=(PendingCommand&&) noexcept = default;
    };
    std::vector<PendingCommand> pending_commands_;  // 每个电机一个待处理命令缓冲区
    
    // CSP模式位置滤波缓冲区（每个电机的位置值）
    std::vector<std::deque<int32_t>> position_filter_buffers_;
    size_t position_filter_size_;  // 滤波器大小，默认3（延迟125us）
    std::vector<bool> position_override_active_;  // 标志：是否从外部覆盖位置（轨迹规划器）
    std::vector<int32_t> idle_input_positions_;  // 静止时输入到buffer的位置值（保持位置）
    
    // CSV模式速度控制
    std::vector<bool> velocity_override_active_;  // 标志：是否从外部覆盖速度（轨迹规划器）
    std::vector<int32_t> idle_input_velocities_;  // 静止时的速度值（通常为0）
    
    // ========== 运行时日志边沿检测缓存 ==========
    std::vector<uint16_t> last_logged_status_words_;
    std::vector<uint16_t> last_logged_control_words_;
    std::vector<uint16_t> last_logged_cia402_states_;
    std::vector<int32_t> last_logged_target_positions_;
    std::vector<int32_t> last_logged_target_velocities_;
    std::vector<int16_t> last_logged_target_torques_;
    std::vector<uint16_t> target_setpoint_log_counters_;

    RuntimeLogger* runtime_logger_ = nullptr;

    // ========== PDO 缓存（RT 写 / ROS 读，经 seqlock 发布一致性快照） ==========
    std::vector<uint16_t> last_status_words_;         // 状态字 (0x6041)
    std::vector<int8_t> last_operation_mode_displays_; // 操作模式显示 (0x6061)
    std::vector<int32_t> last_actual_positions_;      // 实际位置 (0x6064)
    std::vector<int32_t> last_actual_velocities_;     // 实际速度 (0x606C)
    std::vector<int16_t> last_actual_torques_;        // 实际力矩 (0x6077)
    std::vector<int32_t> last_sensor_force_2020_;     // 输出端力矩传感器 (0x2020)
    std::vector<int32_t> last_motor_encoder_2021_;    // 电机端编码器 (0x2021)
    mutable SeqLock pdo_cache_seq_;
    
    // ========== 网关数据缓存（每周期更新，供外部访问） ==========
    struct GatewayData {
        uint32_t can_rx_id;              // CAN接收ID
        uint8_t can_rx_length;          // CAN接收长度
        uint8_t can_rx_data[64];        // CAN接收数据（64字节）
        uint8_t rs485_1_tx_length;       // RS485_1发送长度
        uint8_t rs485_1_tx_data[64];     // RS485_1发送数据（64字节）
        uint8_t rs485_2_tx_length;       // RS485_2发送长度
        uint8_t rs485_2_tx_data[64];     // RS485_2发送数据（64字节）
        
        GatewayData() : can_rx_id(0), can_rx_length(0), 
                       rs485_1_tx_length(0), rs485_2_tx_length(0) {
            memset(can_rx_data, 0, 64);
            memset(rs485_1_tx_data, 0, 64);
            memset(rs485_2_tx_data, 0, 64);
        }
    };
    std::vector<GatewayData> gateway_data_;  // 每个网关一个数据缓存
    
    // 控制字写入缓存和验证机制
    std::vector<uint16_t> pending_control_words_;      // 待写入的控制字（确保cyclic循环会写入）
    std::vector<uint16_t> last_written_control_words_; // 上次实际写入的控制字（用于验证）
    std::vector<bool> control_word_write_pending_;     // 是否有待写入的控制字
    std::vector<int> control_word_write_attempts_;     // 写入尝试次数（用于重试）
    
    // 状态标志
    bool initialized_;
    bool activated_;
    bool startup_evidence_passed_{false};
    uint32_t cached_bus_cycle_us_{1000};
    uint64_t external_cmd_watchdog_ns_{0};
    bool csv_cmd_watchdog_{false};
    std::vector<CommandFreshnessState> torque_cmd_freshness_;
    std::vector<CommandFreshnessState> velocity_cmd_freshness_;

    bool safe_output_active_{false};
    std::vector<int32_t> safe_latched_positions_;
    /** 显式 Fault Reset：0xFFFF=无请求；0x00FF=全体；否则为轴索引。 */
    std::atomic<uint16_t> explicit_fault_reset_axis_{0xFFFFU};
    std::atomic<uint16_t> explicit_fault_reset_cycles_{0U};
    
    // PDO 快照用 seqlock；使能命令用 AtomicBool/U8/U16（勿在 RT 路径加阻塞 mutex）
};

} // namespace ethercat_joint

#endif // ETHERCAT_SERVO_IGH_HPP

