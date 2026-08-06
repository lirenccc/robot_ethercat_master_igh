/**
 * @file ethercat_servo.cpp
 * @brief IgH 后端 EtherCAT 伺服实现
 */

#include "ethercat_joint/servo/ethercat_servo_igh.hpp"
#include "ethercat_joint/master/igh/igh_master_config.hpp"
#include "ethercat_joint/master/igh/igh_master_runtime.hpp"
#include "ethercat_joint/master/igh/safe_output.hpp"
#include "ethercat_joint/motor/csp_interpolation_period.hpp"
#include "ethercat_joint/motor/motor_kinematics.hpp"
#include "ethercat_joint/motor/motor_profile.hpp"
#include "ethercat_joint/util/pdo_cache_helpers.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <map>
#include <thread>
#include <time.h>
#include <unistd.h>

namespace ethercat_joint {
namespace {

constexpr uint32_t kGatewayVendorId = 0x00000130;
constexpr uint32_t kGatewayProductCode = 0x01300060;
// 三木禾 SJD-17：不读 SDO 运动学，编码器在减速器输出端（19bit=524288，位置减速比 1）
constexpr uint32_t kSjd17VendorId = 0x000009CF;
constexpr uint32_t kSjd17ProductCode = 0x00010001;

uint16_t g_send_data_wc = 0;
uint8_t g_send_data_wc_state = 0;

// Dual-domain: first PDO in each domain legitimately has byte offset 0.
// Use UINT_MAX as "not registered"; never treat 0 as missing.
constexpr unsigned int kPdoOffsetUnset = UINT_MAX;

inline bool isJointModuleMotor(const MotorConfig& cfg)
{
    return cfg.pdo_layout == PdoLayout::JOINT_MODULE;
}

inline bool isCoolDriveJmdtMotor(const MotorConfig& cfg)
{
    return cfg.pdo_layout == PdoLayout::COOLDRIVE_JMDT;
}

inline const char* deviceTypeName(const MotorConfig& cfg) noexcept
{
    if (cfg.vendor_id == kGatewayVendorId && cfg.product_code == kGatewayProductCode) {
        return "网关";
    }
    if (cfg.pdo_layout == PdoLayout::COOLDRIVE_JMDT) {
        return "天机 JMDT 关节模组";
    }
    if (cfg.vendor_id == kSjd17VendorId && cfg.product_code == kSjd17ProductCode) {
        return "三木禾 SJD17 关节模组";
    }
    if (cfg.pdo_layout == PdoLayout::JOINT_MODULE) {
        return "新奇 NH17 关节模组";
    }
    return "电机";
}

const MotorProfile * resolveMotorProfile(const MotorConfig & cfg)
{
    if (!cfg.model_id.empty()) {
        if (const MotorProfile * p = MotorProfileRegistry::findByModelId(cfg.model_id)) {
            return p;
        }
    }
    return MotorProfileRegistry::findByIdentity(cfg.vendor_id, cfg.product_code);
}

/** SYNC0 shift：优先 motor_profile.dc_shift_ns（JMDT=720µs 对齐天机 IgH；JOINT 默认 0）。 */
uint32_t resolveDcShiftNs(const MotorConfig & cfg)
{
    if (const MotorProfile * profile = resolveMotorProfile(cfg)) {
        return profile->dc_shift_ns;
    }
    if (isJointModuleMotor(cfg) || isCoolDriveJmdtMotor(cfg)) {
        return 0U;
    }
    return 100000U;
}

/** DC assign_activate（0x0300=启用 SYNC0；0x0000=关闭 DC）。优先 motor_profile.dc_assign_activate。 */
uint16_t resolveDcAssignActivate(const MotorConfig & cfg)
{
    if (const MotorProfile * profile = resolveMotorProfile(cfg)) {
        return profile->dc_assign_activate;
    }
    return 0x0300U;
}

/** SYNC0 周期（ns）：优先 motor_profile.dc_sync0_ns；0 → 沿用总线周期。
 *  profile 显式值须与总线周期一致，否则由调用方 fail-closed。 */
uint32_t resolveDcSync0Ns(const MotorConfig & cfg, uint32_t bus_cycle_ns)
{
    if (const MotorProfile * profile = resolveMotorProfile(cfg)) {
        if (profile->dc_sync0_ns > 0U) {
            return profile->dc_sync0_ns;
        }
    }
    return bus_cycle_ns;
}

/** profile.dc_sync0_ns 与总线不一致时返回 false（避免 Job 节拍与 SYNC0 脱钩）。 */
bool dcSync0MatchesBus(const MotorConfig & cfg, uint32_t bus_cycle_ns)
{
    if (const MotorProfile * profile = resolveMotorProfile(cfg)) {
        if (profile->dc_sync0_ns > 0U && profile->dc_sync0_ns != bus_cycle_ns) {
            return false;
        }
    }
    return true;
}

inline bool isGateway(const MotorConfig& cfg)
{
    return cfg.vendor_id == kGatewayVendorId && cfg.product_code == kGatewayProductCode;
}

inline double pulseToDegree(int32_t pulse, size_t motor_id)
{
    return MotorKinematics::pulseToDegree(pulse, motor_id);
}

inline double pulsePerSecToDegreePerSec(int32_t pulse_per_sec, size_t motor_id)
{
    return MotorKinematics::pulsePerSecToDegreePerSec(pulse_per_sec, motor_id);
}

inline double rawTorqueToOutputTorque(int16_t torque_raw, size_t motor_id)
{
    return MotorKinematics::rawTorqueToOutputTorque(torque_raw, motor_id);
}

bool uploadSdoUint32(ec_master_t* master, uint16_t slave_pos, uint16_t index, uint8_t subindex,
                       uint32_t& value);
bool uploadSdoU8(ec_master_t* master, uint16_t slave_pos, uint16_t index, uint8_t subindex,
                   uint8_t& value);
bool downloadSdoU8(ec_master_t* master, uint16_t slave_pos, uint16_t index, uint8_t subindex,
                   uint8_t value);

}  // namespace

EtherCATServo::EtherCATServo(unsigned int master_index)
    : master_index_(master_index)
    , master_(nullptr)
    , domain_out_(nullptr)
    , domain_in_(nullptr)
    , domain_out_pd_(nullptr)
    , domain_in_pd_(nullptr)
    , motor_count_(0)
    , position_filter_size_(3)  // 默认3点滤波（延迟125us）
    , initialized_(false)
    , activated_(false)
{
    sync_handler_ = std::make_shared<EtherCATSync>();
    memset(&master_state_, 0, sizeof(master_state_));
    memset(&domain_state_, 0, sizeof(domain_state_));
    memset(&domain_out_state_, 0, sizeof(domain_out_state_));
    memset(&domain_in_state_, 0, sizeof(domain_in_state_));
}

EtherCATServo::~EtherCATServo()
{
    deactivate();
    if (master_) {
        ecrt_release_master(master_);
        master_ = nullptr;
    }
}

bool EtherCATServo::initialize(const std::vector<MotorConfig>& motor_configs)
{
    if (initialized_) {
        std::cerr << "EtherCAT Servo already initialized!" << std::endl;
        return false;
    }
    
    motor_count_ = motor_configs.size();  // 根据传入配置动态确定电机数量
    if (motor_count_ == 0) {
        std::cerr << "No motor configurations provided!" << std::endl;
        return false;
    }
    
    motor_configs_ = motor_configs;

    const IghMasterConfig rt_cfg = IghMasterConfig::fromEnvironment();
    cached_bus_cycle_us_ = rt_cfg.bus_cycle_us;
    external_cmd_watchdog_ns_ =
        static_cast<uint64_t>(rt_cfg.cmd_watchdog_ms) * 1000000ULL;
    csv_cmd_watchdog_ = rt_cfg.csv_cmd_watchdog;
    
    std::cout << "[IgH] init master=" << master_index_
              << " slaves=" << motor_count_ << std::endl;

    // 请求主站
    std::cout << "Requesting EtherCAT master " << master_index_ << "..." << std::endl;
    master_ = ecrt_request_master(master_index_);
    if (!master_) {
        std::cerr << "Failed to request master!" << std::endl;
        return false;
    }
    
    // 单域：输出/输入合入同一个域（LRW），对齐参考实现；双域使天机报 0xFF51
    std::cout << "Creating single domain (LRW out+in)..." << std::endl;
    domain_out_ = ecrt_master_create_domain(master_);
    if (!domain_out_) {
        std::cerr << "Failed to create domain!" << std::endl;
        return false;
    }
    domain_in_ = domain_out_;  // 单域别名：in 条目注册进同一域
    std::cout << "[IgH] single domain (LRW) Tianji 7DoF" << std::endl;

    // 初始化同步模块（周期与 IGH_BUS_CYCLE_US 一致）
    if (sync_handler_ && master_) {
        sync_handler_->initialize(master_, cached_bus_cycle_us_ * 1000U);
    }
    
    // 配置从站（包括网关） 
    slave_configs_.resize(motor_count_);  // ⭐ motor_count_已经包括网关（=8）
    pdo_offsets_.resize(motor_count_);
    gateway_data_.resize(motor_count_);  // 初始化网关数据缓存（每个从站一个，只有网关会使用）
    
    std::cout << "[IgH] configuring " << motor_count_ << " slaves" << std::endl;

    // 主站已占用时 ethercat 命令读不到 VID/PID，此处用 API 校正
    for (size_t i = 0; i < motor_count_; ++i) {
        ec_slave_info_t info{};
        if (ecrt_master_get_slave(master_, motor_configs_[i].position, &info) != 0) {
            continue;
        }
        if (motor_configs_[i].vendor_id != info.vendor_id ||
            motor_configs_[i].product_code != info.product_code) {
            std::cout << "  ⚠ 从站 " << motor_configs_[i].position
                      << " VID/PID 校正: 配置 0x" << std::hex
                      << motor_configs_[i].vendor_id << "/0x" << motor_configs_[i].product_code
                      << " → 实际 0x" << info.vendor_id << "/0x" << info.product_code
                      << std::dec << std::endl;
            motor_configs_[i].vendor_id = info.vendor_id;
            motor_configs_[i].product_code = info.product_code;
        }
    }

    ec_slave_config_t* ref_slave_config = nullptr;
    int ref_slave_index = -1;

    for (size_t i = 0; i < motor_count_; ++i) {
        const auto& cfg = motor_configs_[i];
        
        // 仅通过 VID/PID 识别网关（勿用“最后一个从站”，避免末轴误判）
        const bool is_gateway = (cfg.vendor_id == kGatewayVendorId &&
                                 cfg.product_code == kGatewayProductCode);
        const char* device_type = deviceTypeName(cfg);
        
        // ⭐ 所有从站（包含网关）都必须调用 slave_config
        slave_configs_[i] = ecrt_master_slave_config(
            master_,
            cfg.alias, cfg.position,
            cfg.vendor_id, cfg.product_code
        );
        
        if (!slave_configs_[i]) {
            std::cerr << "[IgH] slave_config failed for " << device_type << " " << i 
                        << " (Pos=" << cfg.position << ", VID=0x" << std::hex << cfg.vendor_id 
                        << ", PID=0x" << cfg.product_code << std::dec << ")" << std::endl;
            return false;
        }
        
        // 只把"第一个电机"作为 DC 参考时钟，网关不作为参考时钟
        if (!is_gateway && ref_slave_config == nullptr) {
            ref_slave_config = slave_configs_[i];
            ref_slave_index = static_cast<int>(i);
        }
        
        // DC：SYNC0 周期优先 motor_profile.dc_sync0_ns（JMDT 需 2ms），否则沿用总线周期。
        // shift 来自 motor_profile.dc_shift_ns（JMDT=720µs 对齐天机 IgH）。
        // profile 显式 SYNC0 须等于 IGH_BUS_CYCLE_US，否则 fail-closed。
        const uint32_t bus_cycle_ns =
            (cached_bus_cycle_us_ > 0U ? cached_bus_cycle_us_ : 1000U) * 1000U;
        if (!dcSync0MatchesBus(cfg, bus_cycle_ns)) {
            const uint32_t sync0 = resolveDcSync0Ns(cfg, bus_cycle_ns);
            std::cerr << "[IgH] DC SYNC0 mismatch axis=" << i
                      << " profile_sync0_us=" << (sync0 / 1000U)
                      << " bus_cycle_us=" << (bus_cycle_ns / 1000U)
                      << " (set IGH_BUS_CYCLE_US to match profile, or set profile dc_sync0_ns=0)"
                      << std::endl;
            return false;
        }
        const uint32_t sync0_ns = resolveDcSync0Ns(cfg, bus_cycle_ns);
        const uint32_t dc_shift_ns = resolveDcShiftNs(cfg);
        const uint16_t dc_assign = resolveDcAssignActivate(cfg);
        ecrt_slave_config_dc(slave_configs_[i], dc_assign, sync0_ns, dc_shift_ns, 0, 0);
        if (i == 0) {
            std::cout << "[IgH] DC assign=0x" << std::hex << dc_assign << std::dec
                      << " SYNC0=" << (sync0_ns / 1000) << "us shift="
                      << (dc_shift_ns / 1000) << "us (all axes)" << std::endl;
        }
    }

    // 选择参考时钟：以第一个从站作为参考时钟（进入OP前调用）
    if (ref_slave_config != nullptr) {
        const int ret = ecrt_master_select_reference_clock(master_, ref_slave_config);
        if (ret < 0) {
            std::cerr << "[IgH] Failed to select reference clock (slave "
                      << ref_slave_index << ")" << std::endl;
        } else {
            std::cout << "[IgH] DC reference clock = slave " << ref_slave_index << std::endl;
        }
    } else {
        std::cerr << "[IgH] Warning: No motor for Reference Clock; fallback slave 0" << std::endl;
        // 如果没找到电机，被迫使用第0个
        ecrt_master_select_reference_clock(master_, slave_configs_[0]);
    }
    
    // 配置 PDO 映射（通过 SDO）
    if (!configurePDOMapping()) {
        std::cerr << "Failed to configure PDO mapping!" << std::endl;
        return false;
    }
    
    // 注册 PDO 条目
    if (!registerPDOEntries()) {
        std::cerr << "Failed to register PDO entries!" << std::endl;
        return false;
    }
    
    // 初始化运行时数据
    enable_requested_.resize(motor_count_, false);
    desired_enable_.resize(motor_count_, false);
    enable_fsm_active_.resize(motor_count_, false);
    enable_fsm_step_.resize(motor_count_, 0);
    enable_fsm_wait_.resize(motor_count_, 0);
    
    // ⭐ 原子类型需要特殊初始化：使用swap避免移动操作
    // 先构造临时vector，然后swap（swap不需要移动元素，只需要交换指针）
    {
        std::vector<std::atomic<uint16_t>> temp(motor_count_);
        for (size_t i = 0; i < motor_count_; ++i) {
            temp[i].store(CONTROL_WORD_SWITCH_ON, std::memory_order_relaxed);  // 设置为0x06
        }
        control_word_states_.swap(temp);
    }
    
    fault_reset_counter_.resize(motor_count_, 0);  // 故障复位计数器
    fault_reset_phase_.resize(motor_count_, 0);    // 故障复位相位
    enable_monitor_counter_.resize(motor_count_, 0);  // 使能监控计数器
    enable_monitor_loss_counter_.resize(motor_count_, 0);
    fault_persist_counter_.resize(motor_count_, 0);
    
    // 默认 CSP：循环 PDO 通讯必须写入有效模式与过程数据，从站才能进入 OP
    current_modes_.resize(motor_count_, OperationMode::CYCLIC_SYNC_POSITION);
    
    // ⭐ 原子类型需要特殊初始化：使用swap避免移动操作
    // std::atomic默认构造会初始化为0，正好是我们需要的初始值
    {
        std::vector<std::atomic<int32_t>> temp_pos(motor_count_);
        target_positions_.swap(temp_pos);
    }
    {
        std::vector<std::atomic<int32_t>> temp_vel(motor_count_);
        target_velocities_.swap(temp_vel);
    }
    {
        std::vector<std::atomic<int16_t>> temp_tor(motor_count_);
        target_torques_.swap(temp_tor);
    }
    
    // ⭐ 初始化待处理命令缓冲区：使用swap避免移动操作
    {
        std::vector<PendingCommand> temp(motor_count_);
        pending_commands_.swap(temp);
    }

    torque_cmd_freshness_.assign(motor_count_, CommandFreshnessState{});
    velocity_cmd_freshness_.assign(motor_count_, CommandFreshnessState{});
    
    // ⭐ 初始化CSP模式位置滤波缓冲区（仅电机需要，网关不需要）
    // motor_count_ 包括网关，但滤波器只用于电机
    position_filter_buffers_.resize(motor_count_);
    position_override_active_.resize(motor_count_, false);
    safe_latched_positions_.assign(motor_count_, 0);
    idle_input_positions_.resize(motor_count_, 0);  // 初始化静止输入位置为0
    
    // ⭐ 初始化CSV模式速度控制
    velocity_override_active_.resize(motor_count_, false);
    idle_input_velocities_.resize(motor_count_, 0);  // 静止时速度为0
    
    // ========== 初始化PDO数据低频缓存 ==========
    last_status_words_.resize(motor_count_, 0);
    last_operation_mode_displays_.resize(motor_count_, 0);
    last_error_codes_.resize(motor_count_, 0);
    fault_reset_cw_.resize(motor_count_, CONTROL_WORD_FAULT_RESET);
    for (size_t i = 0; i < motor_count_; ++i) {
        if (const MotorProfile* p = resolveMotorProfile(motor_configs_[i])) {
            fault_reset_cw_[i] = p->fault_reset_control_word;
        }
    }
    last_actual_positions_.resize(motor_count_, 0);
    last_actual_velocities_.resize(motor_count_, 0);
    last_actual_torques_.resize(motor_count_, 0);
    last_sensor_force_2020_.resize(motor_count_, 0);
    last_motor_encoder_2021_.resize(motor_count_, 0);

    last_logged_status_words_.resize(motor_count_, 0xFFFF);
    last_logged_control_words_.resize(motor_count_, 0xFFFF);
    last_logged_cia402_states_.resize(motor_count_, 0xFFFF);
    last_logged_target_positions_.resize(motor_count_, INT32_MIN);
    last_logged_target_velocities_.resize(motor_count_, INT32_MIN);
    last_logged_target_torques_.resize(motor_count_, INT16_MIN);
    target_setpoint_log_counters_.resize(motor_count_, 0);
    
    // buffer 已废弃：停止状态直接使用 idle_input_positions_，无需滤波
    // position_filter_buffers_ 保留定义但不再使用
    
    // 初始化控制字写入缓存和验证机制
    pending_control_words_.resize(motor_count_, CONTROL_WORD_SWITCH_ON);
    last_written_control_words_.resize(motor_count_, CONTROL_WORD_SWITCH_ON);
    control_word_write_pending_.resize(motor_count_, false);
    control_word_write_attempts_.resize(motor_count_, 0);
    
    // 按电机 model/VID·PID 注入运动学，避免沿用 NH17 默认 (101×65536)
    {
        std::vector<MotorKinematicsParams> params(motor_count_);
        for (size_t i = 0; i < motor_count_; ++i) {
            const MotorProfile * profile = nullptr;
            if (!motor_configs_[i].model_id.empty()) {
                profile = MotorProfileRegistry::findByModelId(motor_configs_[i].model_id);
            }
            if (!profile) {
                profile = MotorProfileRegistry::findByIdentity(
                    motor_configs_[i].vendor_id, motor_configs_[i].product_code);
            }
            params[i] = profile ? profile->kinematics : MotorKinematics::get(i);
        }
        MotorKinematics::setParams(params);
        if (motor_count_ > 0 && !isGateway(motor_configs_[0])) {
            std::cout << "[IgH] kinematics[0]: " << MotorKinematics::describe(0) << std::endl;
        }
    }

    initialized_ = true;
    std::cout << "EtherCAT Servo initialized successfully with " 
              << motor_count_ << " motors." << std::endl;
    return true;
}

void EtherCATServo::processSync(uint64_t app_time_ns)
{
    if (sync_handler_ && activated_) {
        // ⭐ 关键修复：传递当前应用时间（逻辑时间）给 process()，确保时间计算准确
        // app_time_ns 应该是 wakeup_time（逻辑时间），与 setApplicationTime() 中使用的相同
        sync_handler_->process(app_time_ns);
    }
}

bool EtherCATServo::activate()
{
    if (!initialized_) {
        std::cerr << "Cannot activate: not initialized!" << std::endl;
        return false;
    }
    
    if (activated_) {
        std::cout << "EtherCAT Servo already activated, skipping activation..." << std::endl;
        return true;
    }

    // 0x60C2 mailbox SDO while still PREOP — must not run after activate without Job.
    const bool sdo_gate_ok = verifyInterpolationPeriodGate();

    std::cout << "Activating master..." << std::endl;
    if (ecrt_master_activate(master_)) {
        std::cerr << "Failed to activate master!" << std::endl;
        return false;
    }

    // 对齐天机：activate 后立刻 send 一次；进 OP 交给后续 Job 周期。
    ecrt_master_send(master_);

    domain_out_pd_ = ecrt_domain_data(domain_out_);
    if (!domain_out_pd_) {
        std::cerr << "Failed to get output domain data!" << std::endl;
        return false;
    }
    domain_in_pd_ = domain_out_pd_;  // 单域：输入/输出同一缓冲

    for (size_t i = 0; i < motor_count_; ++i) {
        enable_requested_[i] = false;
        control_word_states_[i].store(CONTROL_WORD_SWITCH_ON, std::memory_order_relaxed);
    }

    activated_ = true;

    std::cout << "[IgH] activated domains out/in="
              << ecrt_domain_size(domain_out_) << "/"
              << ecrt_domain_size(domain_in_) << " B" << std::endl;

    ec_master_state_t ms;
    ecrt_master_state(master_, &ms);
    std::cout << "[IgH] master link=" << (ms.link_up ? "up" : "down")
              << " slaves=" << ms.slaves_responding
              << " AL=0x" << std::hex << static_cast<int>(ms.al_states) << std::dec
              << std::endl;
    master_state_ = ms;

    // PDO registration evidence only (offsets). Interpolation SDO already ran pre-activate.
    bool pdo_ok = true;
    for (size_t i = 0; i < motor_count_; ++i) {
        if (!verifyPdoEvidence(i)) {
            pdo_ok = false;
            break;
        }
    }
    startup_evidence_passed_ = sdo_gate_ok && pdo_ok;
    if (startup_evidence_passed_) {
        std::cout << "[IgH] startup evidence gate PASSED" << std::endl;
    } else {
        std::cerr << "[IgH] startup evidence gate FAILED"
                  << " (sdo=" << (sdo_gate_ok ? 1 : 0)
                  << " pdo=" << (pdo_ok ? 1 : 0)
                  << ") → observation-only" << std::endl;
    }

    return true;
}


bool EtherCATServo::dcSyncWarmup(uint32_t duration_ms)
{
    if (!initialized_ || !master_ || !domain_out_pd_ || !domain_in_pd_) {
        return false;
    }

    const uint32_t cycle_us =
        cached_bus_cycle_us_ > 0U ? cached_bus_cycle_us_ : 1000U;
    const uint32_t cycle_ns = cycle_us * 1000U;
    const uint32_t settle_cycles =
        duration_ms > 0U ? (duration_ms * 1000U / cycle_us) : 1U;

    std::cout << "[IgH] DC+PDO bootstrap start ("
              << settle_cycles << " cycles × " << cycle_us
              << " µs = ~" << duration_ms << " ms)..." << std::endl;

    // Bootstrap PDO+DC 循环：对齐 robotic-joint-control activate。
    // 从站 PREOP→SAFEOP→OP 需要同时接收 PDO 数据和 DC sync 帧，
    // 否则 SM watchdog (0x001B) 或 PLL error (0x0032) 导致状态转换失败。
    // 不调用 sendData()（会跑 CiA402 FSM），直接写安全 PDO 值。
    uint64_t app_time = sync_handler_ ? sync_handler_->getMonotonicTime() : getMonotonicTimeNs();
    for (uint32_t n = 0; n < settle_cycles; ++n) {
        app_time += cycle_ns;

        // 1. 接收并处理双域 PDO
        ecrt_master_receive(master_);
        ecrt_domain_process(domain_out_);

        // 2. 写安全 PDO 值 —— 控制字=0x06 Shutdown（NH17 文档：进入 OP 前控制字不可为 0）、
        //    操作模式=CSP、目标=空闲位置
        for (size_t i = 0; i < motor_count_; ++i) {
            const auto& offsets = pdo_offsets_[i];
            if (isGateway(motor_configs_[i])) {
                continue;  // 网关跳过（无关节 PDO）
            }
            EC_WRITE_U16(domain_out_pd_ + offsets.control_word, CONTROL_WORD_SWITCH_ON);
            if (offsets.operation_mode != 0) {
                EC_WRITE_S8(domain_out_pd_ + offsets.operation_mode, 8);  // CSP
            }
            int32_t idle_pos = idle_input_positions_[i];
            if (offsets.target_position != 0) {
                EC_WRITE_S32(domain_out_pd_ + offsets.target_position, idle_pos);
            }
            if (offsets.target_velocity != 0) {
                EC_WRITE_S32(domain_out_pd_ + offsets.target_velocity, 0);
            }
            if (offsets.target_torque != 0) {
                EC_WRITE_S16(domain_out_pd_ + offsets.target_torque, 0);
            }
        }

        // 3. DC BurstBulk 同步（12 帧/周期，对齐 EC-Master DCM）
        ecrt_master_application_time(master_, app_time);
        constexpr int kSyncBurstCount = 12;
        for (int b = 0; b < kSyncBurstCount; ++b) {
            ecrt_master_sync_reference_clock(master_);
            ecrt_master_sync_slave_clocks(master_);
        }

        // 4. 排队并发送
        ecrt_domain_queue(domain_out_);
        ecrt_master_send(master_);

        if (n % 100 == 0 || n == settle_cycles - 1) {
            ec_master_state_t ms;
            ecrt_master_state(master_, &ms);
            std::cout << "  bootstrap " << n << "/" << settle_cycles
                      << " slaves_responding=" << ms.slaves_responding
                      << " AL=0x" << std::hex << static_cast<int>(ms.al_states)
                      << std::dec << " link=" << (ms.link_up ? "up" : "DOWN")
                      << std::endl;
        }

        timespec ts{0, static_cast<long>(cycle_ns)};
        nanosleep(&ts, nullptr);
    }

    bool dc_locked = false;
    {
        ec_master_state_t ms;
        ecrt_master_state(master_, &ms);
        dc_locked = (ms.slaves_responding > 0 &&
                     (ms.al_states & EC_AL_STATE_SAFEOP));
        std::cout << "[IgH] DC+PDO bootstrap done: slaves=" << ms.slaves_responding
                  << " AL=0x" << std::hex << static_cast<int>(ms.al_states)
                  << std::dec << " dc_locked=" << (dc_locked ? "yes" : "no")
                  << std::endl;
    }

    return dc_locked;
}


bool EtherCATServo::verifyPdoEvidence(size_t motor_id) const
{
    if (motor_id >= motor_count_) {
        return false;
    }
    if (isGateway(motor_configs_[motor_id])) {
        return true;
    }

    const PDOOffsets& off = pdo_offsets_[motor_id];
    const bool joint =
        motor_configs_[motor_id].pdo_layout == PdoLayout::JOINT_MODULE ||
        motor_configs_[motor_id].pdo_layout == PdoLayout::UNKNOWN ||
        isJointModuleMotor(motor_configs_[motor_id]) ||
        isCoolDriveJmdtMotor(motor_configs_[motor_id]);

    if (off.status_word == kPdoOffsetUnset) {
        std::cerr << "[IgH] PDO_EVIDENCE axis=" << motor_id
                  << " result=fail missing 0x6041 status_word" << std::endl;
        return false;
    }
    if (joint && (off.actual_position == kPdoOffsetUnset ||
                  off.target_position == kPdoOffsetUnset)) {
        std::cerr << "[IgH] PDO_EVIDENCE axis=" << motor_id
                  << " result=fail missing 0x6064/0x607A for joint module"
                  << std::endl;
        return false;
    }
    return true;
}

bool EtherCATServo::verifyInterpolationPeriodGate()
{
    const uint32_t bus_cycle_us = cached_bus_cycle_us_ > 0U
        ? cached_bus_cycle_us_
        : IghMasterRuntime::instance().busCycleUs();
    const uint64_t bus_cycle_ns = static_cast<uint64_t>(bus_cycle_us) * 1000ULL;
    const auto expected = encodeCspInterpolationPeriod(bus_cycle_ns);

    for (size_t i = 0; i < motor_count_; ++i) {
        if (isGateway(motor_configs_[i])) {
            continue;
        }
        const MotorProfile* profile = nullptr;
        if (!motor_configs_[i].model_id.empty()) {
            profile = MotorProfileRegistry::findByModelId(motor_configs_[i].model_id);
        }
        if (!profile) {
            profile = MotorProfileRegistry::findByIdentity(
                motor_configs_[i].vendor_id, motor_configs_[i].product_code);
        }

        const bool require_gate =
            profile ? profile->require_interpolation_period_gate : true;
        if (!require_gate) {
            continue;
        }

        if (!expected) {
            std::cerr << "[IgH] SDO_GATE axis=" << i
                      << " result=fail bus_cycle_ns=" << bus_cycle_ns
                      << " not representable by 0x60C2" << std::endl;
            return false;
        }

        const uint16_t slave_pos = motor_configs_[i].position;
        uint32_t value_u = 0;
        uint32_t exp_u = 0;
        bool ok_v = uploadSdoUint32(master_, slave_pos, 0x60C2, 1, value_u);
        bool ok_e = uploadSdoUint32(master_, slave_pos, 0x60C2, 2, exp_u);
        uint8_t actual_value = 0;
        uint8_t actual_exp = 0;
        if (ok_v && ok_e) {
            actual_value = static_cast<uint8_t>(value_u & 0xFFU);
            actual_exp = static_cast<uint8_t>(exp_u & 0xFFU);
        } else {
            // 部分模组 0x60C2 为 U8（新奇 NH17 ENI 下载单字节）；按 U8 重试
            ok_v = uploadSdoU8(master_, slave_pos, 0x60C2, 1, actual_value);
            ok_e = uploadSdoU8(master_, slave_pos, 0x60C2, 2, actual_exp);
        }
        if (!ok_v || !ok_e) {
            std::cerr << "[IgH] SDO_GATE axis=" << i
                      << " index=0x60C2 result=fail upload" << std::endl;
            return false;
        }

        const int8_t actual_exp_s = static_cast<int8_t>(actual_exp);
        bool match = interpolationPeriodMatchesBus(
            actual_value, actual_exp_s, bus_cycle_ns);
        if (!match) {
            // 不匹配时按 ENI 方式下载期望值（0x60C2:1=value、0x60C2:2=exponent，U8），再回读确认。
            const bool dl_v = downloadSdoU8(master_, slave_pos, 0x60C2, 1, expected->value);
            const bool dl_e = downloadSdoU8(
                master_, slave_pos, 0x60C2, 2,
                static_cast<uint8_t>(expected->exponent));
            if (!dl_v || !dl_e) {
                std::cerr << "[IgH] SDO_GATE axis=" << i
                          << " index=0x60C2 result=fail download"
                          << " expected_value=" << static_cast<unsigned>(expected->value)
                          << " expected_exponent=" << static_cast<int>(expected->exponent)
                          << std::endl;
                return false;
            }
            uint8_t re_v = 0;
            uint8_t re_e = 0;
            const bool rv = uploadSdoU8(master_, slave_pos, 0x60C2, 1, re_v);
            const bool re_ok = uploadSdoU8(master_, slave_pos, 0x60C2, 2, re_e);
            match = rv && re_ok &&
                re_v == expected->value &&
                static_cast<int8_t>(re_e) == expected->exponent;
            if (!match) {
                std::cerr << "[IgH] SDO_GATE axis=" << i
                          << " result=fail interpolation period mismatch after download"
                          << " actual_value=" << static_cast<unsigned>(actual_value)
                          << " actual_exponent=" << static_cast<int>(actual_exp_s)
                          << " bus_cycle_us=" << bus_cycle_us
                          << std::endl;
                return false;
            }
        }
    }
    return true;
}

bool EtherCATServo::runStartupEvidenceGate()
{
    startup_evidence_passed_ = false;
    for (size_t i = 0; i < motor_count_; ++i) {
        if (!verifyPdoEvidence(i)) {
            std::cerr << "[IgH] startup evidence gate FAILED (PDO)"
                      << " → observation-only" << std::endl;
            return false;
        }
    }
    if (!verifyInterpolationPeriodGate()) {
        std::cerr << "[IgH] startup evidence gate FAILED (SDO_GATE 0x60C2)"
                  << " → observation-only" << std::endl;
        return false;
    }
    startup_evidence_passed_ = true;
    std::cout << "[IgH] startup evidence gate PASSED" << std::endl;
    return true;
}


bool EtherCATServo::configurePDOMapping()
{
    std::cout << "Configuring PDO mapping via EtherCAT API..." << std::endl;

    for (size_t i = 0; i < motor_count_; ++i) {
        const auto& cfg = motor_configs_[i];
        
        bool is_gateway = (cfg.vendor_id == kGatewayVendorId && cfg.product_code == kGatewayProductCode);
        bool is_joint_module = isJointModuleMotor(cfg);

        // 关节模组 JOINT_MODULE：IgH 手册默认 0x1600/0x1A00（SJD17 与 IgH 新奇共用此布局）
        if (is_joint_module) {
            const bool is_sjd17 = (cfg.vendor_id == kSjd17VendorId &&
                                       cfg.product_code == kSjd17ProductCode);
            std::cout << "  ▶ " << (is_sjd17 ? "SJD17" : "Xinqi")
                      << " JOINT_MODULE PDO 0x1600/0x1A00"
                      << (is_sjd17 ? " (14B Rx / 22B Tx + 0x2020/0x2021)" : " (14B Rx / 16B Tx + 0x603F)")
                      << std::endl;

            static const ec_pdo_entry_info_t joint_rx_entries[] = {
                {0x6040, 0x00, 16},
                {0x607A, 0x00, 32},
                {0x60FF, 0x00, 32},
                {0x6071, 0x00, 16},
                {0x6060, 0x00, 8},
                {0x0000, 0x00, 8},
            };
            // 新奇：16B Tx（0x6041/6064/606C/6077/6061/603F + Gap），含 0x603F 错误码
            static const ec_pdo_entry_info_t joint_tx_entries_xinqi[] = {
                {0x6041, 0x00, 16},
                {0x6064, 0x00, 32},
                {0x606C, 0x00, 32},
                {0x6077, 0x00, 16},
                {0x6061, 0x00, 8},
                {0x603F, 0x00, 16},
                {0x0000, 0x00, 8},
            };
            // SJD17：22B Tx，含 0x2020 输出端力矩传感器 + 0x2021 电机端编码器
            static const ec_pdo_entry_info_t joint_tx_entries_sjd17[] = {
                {0x6041, 0x00, 16},
                {0x6064, 0x00, 32},
                {0x606C, 0x00, 32},
                {0x6077, 0x00, 16},
                {0x6061, 0x00, 8},
                {0x2020, 0x00, 32},
                {0x2021, 0x00, 32},
                {0x0000, 0x00, 8},
            };
            static const ec_pdo_info_t joint_rx_pdos[] = {
                {0x1600, 6, joint_rx_entries},
            };
            static const ec_pdo_info_t joint_tx_pdos_xinqi[] = {
                {0x1A00, 7, joint_tx_entries_xinqi},
            };
            static const ec_pdo_info_t joint_tx_pdos_sjd17[] = {
                {0x1A00, 8, joint_tx_entries_sjd17},
            };
            const ec_pdo_info_t* joint_tx_pdos =
                is_sjd17 ? joint_tx_pdos_sjd17 : joint_tx_pdos_xinqi;
            const ec_sync_info_t joint_syncs[] = {
                {2, EC_DIR_OUTPUT, 1, joint_rx_pdos, EC_WD_ENABLE},
                {3, EC_DIR_INPUT,  1, joint_tx_pdos, EC_WD_DISABLE},
                {0xFF, EC_DIR_INVALID, 0, nullptr, EC_WD_DISABLE},
            };

            if (ecrt_slave_config_pdos(slave_configs_[i], EC_END, joint_syncs) != 0) {
                std::cerr << "❌ Failed to configure PDOs for joint module " << i << std::endl;
                return false;
            }

            std::cout << "  ✓ RxPDO (0x1600): 6040/607A/60FF/6071/6060 + padding" << std::endl;
            if (is_sjd17) {
                std::cout << "  ✓ TxPDO (0x1A00): 6041/6064/606C/6077/6061/2020/2021 + padding" << std::endl;
            } else {
                std::cout << "  ✓ TxPDO (0x1A00): 6041/6064/606C/6077/6061/603F + padding" << std::endl;
            }
            continue;
        }

        if (isCoolDriveJmdtMotor(cfg)) {
            if (i == 0) {
                std::cout << "[IgH] CoolDrive JMDT PDO 0x1600/0x1A00 "
                          << "(Rx:6040/6060/5FFE/607A/60FF/6071; Tx:+0x310B)" << std::endl;
            }
            static const ec_pdo_entry_info_t jmdt_rx_entries[] = {
                {0x6040, 0x00, 16},
                {0x6060, 0x00, 8},
                {0x5FFE, 0x00, 8},
                {0x607A, 0x00, 32},
                {0x60FF, 0x00, 32},
                {0x6071, 0x00, 16},
            };
            static const ec_pdo_entry_info_t jmdt_tx_entries[] = {
                {0x6041, 0x00, 16},
                {0x6061, 0x00, 8},
                {0x5FFE, 0x00, 8},
                {0x6064, 0x00, 32},
                {0x606C, 0x00, 32},
                {0x6077, 0x00, 16},
                {0x310B, 0x00, 32},
            };
            static const ec_pdo_info_t jmdt_rx_pdos[] = {
                {0x1600, 6, jmdt_rx_entries},
            };
            static const ec_pdo_info_t jmdt_tx_pdos[] = {
                {0x1A00, 7, jmdt_tx_entries},
            };
            const ec_sync_info_t jmdt_syncs[] = {
                {2, EC_DIR_OUTPUT, 1, jmdt_rx_pdos, EC_WD_ENABLE},
                {3, EC_DIR_INPUT,  1, jmdt_tx_pdos, EC_WD_DISABLE},
                {0xFF, EC_DIR_INVALID, 0, nullptr, EC_WD_DISABLE},
            };
            if (ecrt_slave_config_pdos(slave_configs_[i], EC_END, jmdt_syncs) != 0) {
                std::cerr << "[IgH] Failed to configure PDOs for CoolDrive JMDT "
                          << i << std::endl;
                return false;
            }
            continue;
        }
        
        if (is_gateway) {
            std::cout << "========================================" << std::endl;
            std::cout << "配置网关 PDO (从站 " << i << ")..." << std::endl;
            
            // ⭐ 网关的完整 PDO 配置（参考根目录实现）
            // 使用 ESI 生成的完整映射，包含 CAN/CANFD/RS485 等
            static const ec_pdo_entry_info_t gateway_pdo_entries[] = {
                // 0x1600 - RxPDO1: 0x2000 + 0x2002 + 0x2004
                {0x2000, 0x01, 32},
                {0x2000, 0x02, 8},
                {0x2000, 0x03, 8},
                {0x2000, 0x04, 8},
                {0x2000, 0x05, 8},
                {0x2000, 0x06, 8},
                {0x2000, 0x07, 8},
                {0x2000, 0x08, 8},
                {0x2000, 0x09, 8},
                {0x2000, 0x0a, 8},
                {0x2000, 0x0b, 8},
                {0x2000, 0x0c, 8},
                {0x2000, 0x0d, 8},
                {0x2000, 0x0e, 8},
                {0x2000, 0x0f, 8},
                {0x2000, 0x10, 8},
                {0x2000, 0x11, 8},
                {0x2000, 0x12, 8},
                {0x2000, 0x13, 8},
                {0x2000, 0x14, 8},
                {0x2000, 0x15, 8},
                {0x2000, 0x16, 8},
                {0x2000, 0x17, 8},
                {0x2000, 0x18, 8},
                {0x2000, 0x19, 8},
                {0x2000, 0x1a, 8},
                {0x2000, 0x1b, 8},
                {0x2000, 0x1c, 8},
                {0x2000, 0x1d, 8},
                {0x2000, 0x1e, 8},
                {0x2000, 0x1f, 8},
                {0x2000, 0x20, 8},
                {0x2000, 0x21, 8},
                {0x2000, 0x22, 8},
                {0x2000, 0x23, 8},
                {0x2000, 0x24, 8},
                {0x2000, 0x25, 8},
                {0x2000, 0x26, 8},
                {0x2000, 0x27, 8},
                {0x2000, 0x28, 8},
                {0x2000, 0x29, 8},
                {0x2000, 0x2a, 8},
                {0x2000, 0x2b, 8},
                {0x2000, 0x2c, 8},
                {0x2000, 0x2d, 8},
                {0x2000, 0x2e, 8},
                {0x2000, 0x2f, 8},
                {0x2000, 0x30, 8},
                {0x2000, 0x31, 8},
                {0x2000, 0x32, 8},
                {0x2000, 0x33, 8},
                {0x2000, 0x34, 8},
                {0x2000, 0x35, 8},
                {0x2000, 0x36, 8},
                {0x2000, 0x37, 8},
                {0x2000, 0x38, 8},
                {0x2000, 0x39, 8},
                {0x2000, 0x3a, 8},
                {0x2000, 0x3b, 8},
                {0x2000, 0x3c, 8},
                {0x2000, 0x3d, 8},
                {0x2000, 0x3e, 8},
                {0x2000, 0x3f, 8},
                {0x2000, 0x40, 8},
                {0x2000, 0x41, 8},
                {0x2000, 0x42, 8},
                {0x2002, 0x01, 8},
                {0x2002, 0x02, 8},
                {0x2002, 0x03, 8},
                {0x2002, 0x04, 8},
                {0x2002, 0x05, 8},
                {0x2002, 0x06, 8},
                {0x2002, 0x07, 8},
                {0x2002, 0x08, 8},
                {0x2002, 0x09, 8},
                {0x2002, 0x0a, 8},
                {0x2002, 0x0b, 8},
                {0x2002, 0x0c, 8},
                {0x2002, 0x0d, 8},
                {0x2002, 0x0e, 8},
                {0x2002, 0x0f, 8},
                {0x2002, 0x10, 8},
                {0x2002, 0x11, 8},
                {0x2002, 0x12, 8},
                {0x2002, 0x13, 8},
                {0x2002, 0x14, 8},
                {0x2002, 0x15, 8},
                {0x2002, 0x16, 8},
                {0x2002, 0x17, 8},
                {0x2002, 0x18, 8},
                {0x2002, 0x19, 8},
                {0x2002, 0x1a, 8},
                {0x2002, 0x1b, 8},
                {0x2002, 0x1c, 8},
                {0x2002, 0x1d, 8},
                {0x2002, 0x1e, 8},
                {0x2002, 0x1f, 8},
                {0x2002, 0x20, 8},
                {0x2002, 0x21, 8},
                {0x2002, 0x22, 8},
                {0x2002, 0x23, 8},
                {0x2002, 0x24, 8},
                {0x2002, 0x25, 8},
                {0x2002, 0x26, 8},
                {0x2002, 0x27, 8},
                {0x2002, 0x28, 8},
                {0x2002, 0x29, 8},
                {0x2002, 0x2a, 8},
                {0x2002, 0x2b, 8},
                {0x2002, 0x2c, 8},
                {0x2002, 0x2d, 8},
                {0x2002, 0x2e, 8},
                {0x2002, 0x2f, 8},
                {0x2002, 0x30, 8},
                {0x2002, 0x31, 8},
                {0x2002, 0x32, 8},
                {0x2002, 0x33, 8},
                {0x2002, 0x34, 8},
                {0x2002, 0x35, 8},
                {0x2002, 0x36, 8},
                {0x2002, 0x37, 8},
                {0x2002, 0x38, 8},
                {0x2002, 0x39, 8},
                {0x2002, 0x3a, 8},
                {0x2002, 0x3b, 8},
                {0x2002, 0x3c, 8},
                {0x2002, 0x3d, 8},
                {0x2002, 0x3e, 8},
                {0x2002, 0x3f, 8},
                {0x2002, 0x40, 8},
                {0x2002, 0x41, 8},
                {0x2004, 0x01, 8},
                {0x2004, 0x02, 8},
                {0x2004, 0x03, 8},
                {0x2004, 0x04, 8},
                {0x2004, 0x05, 8},
                {0x2004, 0x06, 8},
                {0x2004, 0x07, 8},
                {0x2004, 0x08, 8},
                {0x2004, 0x09, 8},
                {0x2004, 0x0a, 8},
                {0x2004, 0x0b, 8},
                {0x2004, 0x0c, 8},
                {0x2004, 0x0d, 8},
                {0x2004, 0x0e, 8},
                {0x2004, 0x0f, 8},
                {0x2004, 0x10, 8},
                {0x2004, 0x11, 8},
                {0x2004, 0x12, 8},
                {0x2004, 0x13, 8},
                {0x2004, 0x14, 8},
                {0x2004, 0x15, 8},
                {0x2004, 0x16, 8},
                {0x2004, 0x17, 8},
                {0x2004, 0x18, 8},
                {0x2004, 0x19, 8},
                {0x2004, 0x1a, 8},
                {0x2004, 0x1b, 8},
                {0x2004, 0x1c, 8},
                {0x2004, 0x1d, 8},
                {0x2004, 0x1e, 8},
                {0x2004, 0x1f, 8},
                {0x2004, 0x20, 8},
                {0x2004, 0x21, 8},
                {0x2004, 0x22, 8},
                {0x2004, 0x23, 8},
                {0x2004, 0x24, 8},
                {0x2004, 0x25, 8},
                {0x2004, 0x26, 8},
                {0x2004, 0x27, 8},
                {0x2004, 0x28, 8},
                {0x2004, 0x29, 8},
                {0x2004, 0x2a, 8},
                {0x2004, 0x2b, 8},
                {0x2004, 0x2c, 8},
                {0x2004, 0x2d, 8},
                {0x2004, 0x2e, 8},
                {0x2004, 0x2f, 8},
                {0x2004, 0x30, 8},
                {0x2004, 0x31, 8},
                {0x2004, 0x32, 8},
                {0x2004, 0x33, 8},
                {0x2004, 0x34, 8},
                {0x2004, 0x35, 8},
                {0x2004, 0x36, 8},
                {0x2004, 0x37, 8},
                {0x2004, 0x38, 8},
                {0x2004, 0x39, 8},
                {0x2004, 0x3a, 8},
                {0x2004, 0x3b, 8},
                {0x2004, 0x3c, 8},
                {0x2004, 0x3d, 8},
                {0x2004, 0x3e, 8},
                {0x2004, 0x3f, 8},
                {0x2004, 0x40, 8},
                {0x2004, 0x41, 8},

                // 0x1A00 - TxPDO1: 0x2001 + 0x2007 + 0x2003
                {0x2001, 0x01, 32},
                {0x2001, 0x02, 8},
                {0x2001, 0x03, 8},
                {0x2001, 0x04, 8},
                {0x2001, 0x05, 8},
                {0x2001, 0x06, 8},
                {0x2001, 0x07, 8},
                {0x2001, 0x08, 8},
                {0x2001, 0x09, 8},
                {0x2001, 0x0a, 8},
                {0x2001, 0x0b, 8},
                {0x2001, 0x0c, 8},
                {0x2001, 0x0d, 8},
                {0x2001, 0x0e, 8},
                {0x2001, 0x0f, 8},
                {0x2001, 0x10, 8},
                {0x2001, 0x11, 8},
                {0x2001, 0x12, 8},
                {0x2001, 0x13, 8},
                {0x2001, 0x14, 8},
                {0x2001, 0x15, 8},
                {0x2001, 0x16, 8},
                {0x2001, 0x17, 8},
                {0x2001, 0x18, 8},
                {0x2001, 0x19, 8},
                {0x2001, 0x1a, 8},
                {0x2001, 0x1b, 8},
                {0x2001, 0x1c, 8},
                {0x2001, 0x1d, 8},
                {0x2001, 0x1e, 8},
                {0x2001, 0x1f, 8},
                {0x2001, 0x20, 8},
                {0x2001, 0x21, 8},
                {0x2001, 0x22, 8},
                {0x2001, 0x23, 8},
                {0x2001, 0x24, 8},
                {0x2001, 0x25, 8},
                {0x2001, 0x26, 8},
                {0x2001, 0x27, 8},
                {0x2001, 0x28, 8},
                {0x2001, 0x29, 8},
                {0x2001, 0x2a, 8},
                {0x2001, 0x2b, 8},
                {0x2001, 0x2c, 8},
                {0x2001, 0x2d, 8},
                {0x2001, 0x2e, 8},
                {0x2001, 0x2f, 8},
                {0x2001, 0x30, 8},
                {0x2001, 0x31, 8},
                {0x2001, 0x32, 8},
                {0x2001, 0x33, 8},
                {0x2001, 0x34, 8},
                {0x2001, 0x35, 8},
                {0x2001, 0x36, 8},
                {0x2001, 0x37, 8},
                {0x2001, 0x38, 8},
                {0x2001, 0x39, 8},
                {0x2001, 0x3a, 8},
                {0x2001, 0x3b, 8},
                {0x2001, 0x3c, 8},
                {0x2001, 0x3d, 8},
                {0x2001, 0x3e, 8},
                {0x2001, 0x3f, 8},
                {0x2001, 0x40, 8},
                {0x2001, 0x41, 8},
                {0x2001, 0x42, 8},
                {0x2007, 0x01, 8},
                {0x2003, 0x01, 8},
                {0x2003, 0x02, 8},
                {0x2003, 0x03, 8},
                {0x2003, 0x04, 8},
                {0x2003, 0x05, 8},
                {0x2003, 0x06, 8},
                {0x2003, 0x07, 8},
                {0x2003, 0x08, 8},
                {0x2003, 0x09, 8},
                {0x2003, 0x0a, 8},
                {0x2003, 0x0b, 8},
                {0x2003, 0x0c, 8},
                {0x2003, 0x0d, 8},
                {0x2003, 0x0e, 8},
                {0x2003, 0x0f, 8},
                {0x2003, 0x10, 8},
                {0x2003, 0x11, 8},
                {0x2003, 0x12, 8},
                {0x2003, 0x13, 8},
                {0x2003, 0x14, 8},
                {0x2003, 0x15, 8},
                {0x2003, 0x16, 8},
                {0x2003, 0x17, 8},
                {0x2003, 0x18, 8},
                {0x2003, 0x19, 8},
                {0x2003, 0x1a, 8},
                {0x2003, 0x1b, 8},
                {0x2003, 0x1c, 8},
                {0x2003, 0x1d, 8},
                {0x2003, 0x1e, 8},
                {0x2003, 0x1f, 8},
                {0x2003, 0x20, 8},
                {0x2003, 0x21, 8},
                {0x2003, 0x22, 8},
                {0x2003, 0x23, 8},
                {0x2003, 0x24, 8},
                {0x2003, 0x25, 8},
                {0x2003, 0x26, 8},
                {0x2003, 0x27, 8},
                {0x2003, 0x28, 8},
                {0x2003, 0x29, 8},
                {0x2003, 0x2a, 8},
                {0x2003, 0x2b, 8},
                {0x2003, 0x2c, 8},
                {0x2003, 0x2d, 8},
                {0x2003, 0x2e, 8},
                {0x2003, 0x2f, 8},
                {0x2003, 0x30, 8},
                {0x2003, 0x31, 8},
                {0x2003, 0x32, 8},
                {0x2003, 0x33, 8},
                {0x2003, 0x34, 8},
                {0x2003, 0x35, 8},
                {0x2003, 0x36, 8},
                {0x2003, 0x37, 8},
                {0x2003, 0x38, 8},
                {0x2003, 0x39, 8},
                {0x2003, 0x3a, 8},
                {0x2003, 0x3b, 8},
                {0x2003, 0x3c, 8},
                {0x2003, 0x3d, 8},
                {0x2003, 0x3e, 8},
                {0x2003, 0x3f, 8},
                {0x2003, 0x40, 8},
                {0x2003, 0x41, 8},
                {0x2007, 0x01, 8},
                {0x2005, 0x01, 8},
                {0x2005, 0x02, 8},
                {0x2005, 0x03, 8},
                {0x2005, 0x04, 8},
                {0x2005, 0x05, 8},
                {0x2005, 0x06, 8},
                {0x2005, 0x07, 8},
                {0x2005, 0x08, 8},
                {0x2005, 0x09, 8},
                {0x2005, 0x0a, 8},
                {0x2005, 0x0b, 8},
                {0x2005, 0x0c, 8},
                {0x2005, 0x0d, 8},
                {0x2005, 0x0e, 8},
                {0x2005, 0x0f, 8},
                {0x2005, 0x10, 8},
                {0x2005, 0x11, 8},
                {0x2005, 0x12, 8},
                {0x2005, 0x13, 8},
                {0x2005, 0x14, 8},
                {0x2005, 0x15, 8},
                {0x2005, 0x16, 8},
                {0x2005, 0x17, 8},
                {0x2005, 0x18, 8},
                {0x2005, 0x19, 8},
                {0x2005, 0x1a, 8},
                {0x2005, 0x1b, 8},
                {0x2005, 0x1c, 8},
                {0x2005, 0x1d, 8},
                {0x2005, 0x1e, 8},
                {0x2005, 0x1f, 8},
                {0x2005, 0x20, 8},
                {0x2005, 0x21, 8},
                {0x2005, 0x22, 8},
                {0x2005, 0x23, 8},
                {0x2005, 0x24, 8},
                {0x2005, 0x25, 8},
                {0x2005, 0x26, 8},
                {0x2005, 0x27, 8},
                {0x2005, 0x28, 8},
                {0x2005, 0x29, 8},
                {0x2005, 0x2a, 8},
                {0x2005, 0x2b, 8},
                {0x2005, 0x2c, 8},
                {0x2005, 0x2d, 8},
                {0x2005, 0x2e, 8},
                {0x2005, 0x2f, 8},
                {0x2005, 0x30, 8},
                {0x2005, 0x31, 8},
                {0x2005, 0x32, 8},
                {0x2005, 0x33, 8},
                {0x2005, 0x34, 8},
                {0x2005, 0x35, 8},
                {0x2005, 0x36, 8},
                {0x2005, 0x37, 8},
                {0x2005, 0x38, 8},
                {0x2005, 0x39, 8},
                {0x2005, 0x3a, 8},
                {0x2005, 0x3b, 8},
                {0x2005, 0x3c, 8},
                {0x2005, 0x3d, 8},
                {0x2005, 0x3e, 8},
                {0x2005, 0x3f, 8},
                {0x2005, 0x40, 8},
                {0x2005, 0x41, 8},
                {0x2007, 0x01, 8},
            };

            static const ec_pdo_info_t gateway_pdos[] = {
                {0x1600, 66, gateway_pdo_entries + 0},
                {0x1601, 65, gateway_pdo_entries + 66},
                {0x1602, 65, gateway_pdo_entries + 131},
                {0x1A00, 67, gateway_pdo_entries + 196},
                {0x1A01, 66, gateway_pdo_entries + 263},
                {0x1A02, 66, gateway_pdo_entries + 329},
            };

            static const ec_sync_info_t gateway_syncs[] = {
                {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
                {1, EC_DIR_INPUT,  0, NULL, EC_WD_DISABLE},
                {2, EC_DIR_OUTPUT, 3, gateway_pdos + 0, EC_WD_ENABLE},
                {3, EC_DIR_INPUT,  3, gateway_pdos + 3, EC_WD_DISABLE},
                {0xFF, EC_DIR_INVALID, 0, NULL, EC_WD_DISABLE}
            };

            int ret = ecrt_slave_config_pdos(slave_configs_[i], EC_END, gateway_syncs);
            if (ret != 0) {
                std::cerr << "❌ Failed to configure PDOs for gateway" << std::endl;
                return false;
            }
            std::cout << "  ✓ 网关PDO配置成功 (完整映射: CAN/CANFD/RS485)" << std::endl;
            std::cout << "========================================" << std::endl;
            continue;
        }
        
        // 配置电机PDO
        std::cerr << "❌ 未支持的关节模组 PDO 布局: " << cfg.name
                  << " (model=" << cfg.model_id
                  << ", VID=0x" << std::hex << cfg.vendor_id
                  << ", PID=0x" << cfg.product_code << std::dec << ")" << std::endl;
        std::cerr << "  请在 motor_profile.cpp 注册该型号，并设置正确的 motor_model" << std::endl;
        return false;
    }
    
    std::cout << "✓ PDO mapping step completed" << std::endl;
    return true;
}

bool EtherCATServo::registerPDOEntries()
{
    // 单电机工程 PDO 映射（与主工程普通电机保持一致）
    // RxPDO: 0x6040, 0x6060, 0x5FFE, 0x607A, 0x60FF, 0x6071
    // TxPDO: 0x6041, 0x6061, 0x5FFE, 0x6064, 0x606C, 0x6077
    
    // 注册PDO条目（包括网关）
    for (size_t i = 0; i < motor_count_; ++i) {
        const auto& cfg = motor_configs_[i];
        auto& offsets = pdo_offsets_[i];
        
        bool is_gateway = (cfg.vendor_id == kGatewayVendorId && cfg.product_code == kGatewayProductCode);

        if (is_gateway) {
            // ⭐ 网关配置：目前不安装手，只配置基本的PDO映射，不注册CAN/CANFD/RS485的PDO条目
            // ⭐ 如果将来需要安装手，可以启用相应的PDO注册代码
            std::cout << "\n[网关] 配置网关PDO (从站索引 " << i << ", 位置=" << cfg.position 
                      << ", name=" << cfg.name 
                      << ", VID=0x" << std::hex << cfg.vendor_id 
                      << ", PID=0x" << cfg.product_code << std::dec << ")" << std::endl;
            
            // ⭐ 检查从站配置是否成功
            bool is_last_slave = (i == motor_count_ - 1);
            if (!slave_configs_[i]) {
                std::cerr << "❌ 错误: 网关从站 " << i << " 的配置失败，无法注册PDO" << std::endl;
                std::cerr << "   从站配置: VID=0x" << std::hex << cfg.vendor_id 
                          << ", PID=0x" << cfg.product_code 
                          << ", position=" << std::dec << cfg.position << std::endl;
                std::cerr << "   是否最后一个从站: " << (is_last_slave ? "是" : "否") << std::endl;
                return false;
            }
            
            memset(&offsets, 0, sizeof(PDOOffsets));
            
            // ⭐ 即使不安装手，也要注册网关的所有PDO条目（ID、长度、数据字段），以便在循环中持续发送和接收
            // ⭐ 这样网关就能正常工作，只是数据长度和内容都是0
            // ⭐ 注意：网关的PDO映射已经包含了所有字段，必须注册所有字段才能正常工作
            
            // ========== 发送PDO（主站 -> 网关）==========
            // CAN/CANFD发送：注册ID、长度和数据字段
            // CAN TX ID (UINT32) - 0x2000:01
            int ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i],
                    0x2000, 0x01, domain_out_, nullptr);
            if (ret < 0) {
                std::cerr << "❌ Failed to register CAN TX ID (0x2000:01) for gateway " << i << std::endl;
                return false;
            }
            offsets.can_tx_id = ret;
            offsets.canfd_tx_id = ret;  // CANFD复用CAN的偏移量
            std::cout << "  ✓ CAN/CANFD TX ID (0x2000:01): offset=" << ret << std::endl;
            
            // CAN TX length (UINT8) - 0x2000:02
            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i],
                    0x2000, 0x02, domain_out_, nullptr);
            if (ret < 0) {
                std::cerr << "❌ Failed to register CAN TX length (0x2000:02) for gateway " << i << std::endl;
                return false;
            }
            offsets.can_tx_length = ret;
            offsets.canfd_tx_length = ret;  // CANFD复用CAN的偏移量
            std::cout << "  ✓ CAN/CANFD TX length (0x2000:02): offset=" << ret << std::endl;
            
            // CAN TX data (UINT8) - 0x2000:03（第一个数据字节，用于64字节数据）
            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i],
                    0x2000, 0x03, domain_out_, nullptr);
            if (ret < 0) {
                std::cerr << "❌ Failed to register CAN TX data (0x2000:03) for gateway " << i << std::endl;
                return false;
            }
            offsets.can_tx_data = ret;
            offsets.canfd_tx_data = ret;  // CANFD复用CAN的偏移量
            std::cout << "  ✓ CAN/CANFD TX data (0x2000:03): offset=" << ret << " (64 bytes)" << std::endl;
            
            // 注册其余CAN TX数据字节 (0x2000:04 到 0x2000:41，共64字节)
            for (uint8_t subindex = 0x04; subindex <= 0x41; subindex++) {
                ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i],
                        0x2000, subindex, domain_out_, nullptr);
                if (ret < 0) {
                    std::cerr << "❌ Failed to register CAN TX data byte (0x2000:" 
                              << std::hex << (int)subindex << std::dec << ") for gateway " << i << std::endl;
                    return false;
                }
            }
            std::cout << "  ✓ CAN/CANFD TX data bytes (0x2000:03-0x2000:41, 64 bytes) registered" << std::endl;
            
            // RS485_1接收：注册长度和数据字段
            // RS485_1 RX length (UINT8) - 0x2002:01
            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i],
                    0x2002, 0x01, domain_out_, nullptr);
            if (ret < 0) {
                std::cerr << "❌ Failed to register 485_1RX length (0x2002:01) for gateway " << i << std::endl;
                return false;
            }
            offsets.rs485_1_rx_length = ret;
            std::cout << "  ✓ 485_1RX length (0x2002:01): offset=" << ret << std::endl;
            
            // RS485_1 RX data (UINT8) - 0x2002:02（第一个数据字节）
            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i],
                    0x2002, 0x02, domain_out_, nullptr);
            if (ret < 0) {
                std::cerr << "❌ Failed to register 485_1RX data byte (0x2002:02) for gateway " << i << std::endl;
                return false;
            }
            offsets.rs485_1_rx_data = ret;
            std::cout << "  ✓ 485_1RX data byte 0 (0x2002:02): offset=" << ret << std::endl;
            
            // 注册其余RS485_1 RX数据字节 (0x2002:03 到 0x2002:41，共64字节)
            for (uint8_t subindex = 0x03; subindex <= 0x41; subindex++) {
                ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i],
                        0x2002, subindex, domain_out_, nullptr);
                if (ret < 0) {
                    std::cerr << "❌ Failed to register 485_1RX data byte (0x2002:" 
                              << std::hex << (int)subindex << std::dec << ") for gateway " << i << std::endl;
                    return false;
                }
            }
            std::cout << "  ✓ 485_1RX data bytes (0x2002:02-0x2002:41, 64 bytes) registered" << std::endl;
            
            // RS485_2接收：注册长度和数据字段
            // RS485_2 RX length (UINT8) - 0x2004:01
            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i],
                    0x2004, 0x01, domain_out_, nullptr);
            if (ret < 0) {
                std::cerr << "❌ Failed to register 485_2RX length (0x2004:01) for gateway " << i << std::endl;
                return false;
            }
            offsets.rs485_2_rx_length = ret;
            std::cout << "  ✓ 485_2RX length (0x2004:01): offset=" << ret << std::endl;
            
            // RS485_2 RX data (UINT8) - 0x2004:02（第一个数据字节）
            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i],
                    0x2004, 0x02, domain_out_, nullptr);
            if (ret < 0) {
                std::cerr << "❌ Failed to register 485_2RX data byte (0x2004:02) for gateway " << i << std::endl;
                return false;
            }
            offsets.rs485_2_rx_data = ret;
            std::cout << "  ✓ 485_2RX data byte 0 (0x2004:02): offset=" << ret << std::endl;
            
            // 注册其余RS485_2 RX数据字节 (0x2004:03 到 0x2004:41，共64字节)
            for (uint8_t subindex = 0x03; subindex <= 0x41; subindex++) {
                ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i],
                        0x2004, subindex, domain_out_, nullptr);
                if (ret < 0) {
                    std::cerr << "❌ Failed to register 485_2RX data byte (0x2004:" 
                              << std::hex << (int)subindex << std::dec << ") for gateway " << i << std::endl;
                    return false;
                }
            }
            std::cout << "  ✓ 485_2RX data bytes (0x2004:02-0x2004:41, 64 bytes) registered" << std::endl;
            
            // ========== 接收PDO（网关 -> 主站）==========
            // CAN/CANFD接收：注册ID、长度和数据字段
            // CAN RX ID (UINT32) - 0x2001:01
            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i],
                    0x2001, 0x01, domain_in_, nullptr);
            if (ret < 0) {
                std::cerr << "❌ Failed to register CAN RX ID (0x2001:01) for gateway " << i << std::endl;
                return false;
            }
            offsets.can_rx_id = ret;
            offsets.canfd_rx_id = ret;  // CANFD复用CAN的偏移量
            std::cout << "  ✓ CAN/CANFD RX ID (0x2001:01): offset=" << ret << std::endl;
            
            // CAN RX length (UINT8) - 0x2001:02
            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i],
                    0x2001, 0x02, domain_in_, nullptr);
            if (ret < 0) {
                std::cerr << "❌ Failed to register CAN RX length (0x2001:02) for gateway " << i << std::endl;
                return false;
            }
            offsets.can_rx_length = ret;
            offsets.canfd_rx_length = ret;  // CANFD复用CAN的偏移量
            std::cout << "  ✓ CAN/CANFD RX length (0x2001:02): offset=" << ret << std::endl;
            
            // CAN RX data (UINT8) - 0x2001:03（第一个数据字节，用于64字节数据）
            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i],
                    0x2001, 0x03, domain_in_, nullptr);
            if (ret < 0) {
                std::cerr << "❌ Failed to register CAN RX data (0x2001:03) for gateway " << i << std::endl;
                return false;
            }
            offsets.can_rx_data = ret;
            offsets.canfd_rx_data = ret;  // CANFD复用CAN的偏移量
            std::cout << "  ✓ CAN/CANFD RX data (0x2001:03): offset=" << ret << " (64 bytes)" << std::endl;
            
            // 注册其余CAN RX数据字节 (0x2001:04 到 0x2001:42，共64字节)
            for (uint8_t subindex = 0x04; subindex <= 0x42; subindex++) {
                ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i],
                        0x2001, subindex, domain_out_, nullptr);
                if (ret < 0) {
                    std::cerr << "❌ Failed to register CAN RX data byte (0x2001:" 
                              << std::hex << (int)subindex << std::dec << ") for gateway " << i << std::endl;
                    return false;
                }
            }
            std::cout << "  ✓ CAN/CANFD RX data bytes (0x2001:03-0x2001:42, 64 bytes) registered" << std::endl;
            
            // RS485_1发送：注册长度和数据字段
            // RS485_1 TX length (UINT8) - 0x2003:01
            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i],
                    0x2003, 0x01, domain_in_, nullptr);
            if (ret < 0) {
                std::cerr << "❌ Failed to register 485_1TX length (0x2003:01) for gateway " << i << std::endl;
                return false;
            }
            offsets.rs485_1_tx_length = ret;
            std::cout << "  ✓ 485_1TX length (0x2003:01): offset=" << ret << std::endl;
            
            // RS485_1 TX data (UINT8) - 0x2003:02（第一个数据字节）
            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i],
                    0x2003, 0x02, domain_in_, nullptr);
            if (ret < 0) {
                std::cerr << "❌ Failed to register 485_1TX data byte (0x2003:02) for gateway " << i << std::endl;
                return false;
            }
            offsets.rs485_1_tx_data = ret;
            std::cout << "  ✓ 485_1TX data byte 0 (0x2003:02): offset=" << ret << std::endl;
            
            // 注册其余RS485_1 TX数据字节 (0x2003:03 到 0x2003:41，共64字节)
            for (uint8_t subindex = 0x03; subindex <= 0x41; subindex++) {
                ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i],
                        0x2003, subindex, domain_out_, nullptr);
                if (ret < 0) {
                    std::cerr << "❌ Failed to register 485_1TX data byte (0x2003:" 
                              << std::hex << (int)subindex << std::dec << ") for gateway " << i << std::endl;
                    return false;
                }
            }
            std::cout << "  ✓ 485_1TX data bytes (0x2003:02-0x2003:41, 64 bytes) registered" << std::endl;
            
            // RS485_2发送：注册长度和数据字段
            // ⚠️ 注意：某些网关硬件可能不支持0x2005字典，需要先检查是否存在
            // RS485_2 TX length (UINT8) - 0x2005:01
            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i],
                    0x2005, 0x01, domain_in_, nullptr);
            if (ret < 0) {
                // 0x2005字典不存在，跳过RS485_2 TX的注册
                std::cout << "  ⚠️  网关不支持0x2005字典（RS485_2 TX），跳过注册" << std::endl;
                std::cout << "  注意: 0x2005字典不存在，将从PDO映射中移除相关条目" << std::endl;
                offsets.rs485_2_tx_length = 0;
                offsets.rs485_2_tx_data = 0;
            } else {
                offsets.rs485_2_tx_length = ret;
                std::cout << "  ✓ 485_2TX length (0x2005:01): offset=" << ret << std::endl;
                
                // RS485_2 TX data (UINT8) - 0x2005:02（第一个数据字节）
                ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i],
                        0x2005, 0x02, domain_in_, nullptr);
                if (ret < 0) {
                    std::cerr << "  ⚠️  警告: 0x2005:02注册失败，但0x2005:01已成功，可能存在部分子索引缺失" << std::endl;
                    offsets.rs485_2_tx_data = 0;
                } else {
                    offsets.rs485_2_tx_data = ret;
                    std::cout << "  ✓ 485_2TX data byte 0 (0x2005:02): offset=" << ret << std::endl;
                    
                    // 注册其余RS485_2 TX数据字节 (0x2005:03 到 0x2005:41，共64字节)
                    // ⚠️ 如果某些子索引不存在，记录警告但继续尝试其他索引
                    int success_count = 0;
                    int fail_count = 0;
                    for (uint8_t subindex = 0x03; subindex <= 0x41; subindex++) {
                        ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i],
                                0x2005, subindex, domain_out_, nullptr);
                        if (ret < 0) {
                            fail_count++;
                            if (fail_count <= 3) {  // 只显示前3个失败的信息，避免日志过多
                                std::cerr << "  ⚠️  警告: 无法注册 485_2TX data byte (0x2005:" 
                                          << std::hex << (int)subindex << std::dec 
                                          << ")，该子索引可能不存在" << std::endl;
                            }
                        } else {
                            success_count++;
                        }
                    }
                    if (fail_count > 0) {
                        std::cout << "  ⚠️  RS485_2 TX数据字节注册: 成功 " << success_count 
                                  << " 个，失败 " << fail_count << " 个（某些子索引不存在）" << std::endl;
                    } else {
                        std::cout << "  ✓ 485_2TX data bytes (0x2005:03-0x2005:41, 64 bytes) registered" << std::endl;
                    }
                }
            }
            
            std::cout << "  注意: 已注册网关所有PDO条目（ID、长度、数据字段），数据长度和内容将在循环中写入/读取0值" << std::endl;
            std::cout << "Gateway " << i << " PDO registration completed (all fields registered, data = 0)." << std::endl;
            continue;  // 跳过后续的电机PDO注册
        }
        
        // ⭐ 关节模组 PDO 注册（IgH JOINT_MODULE；SJD17 ENI 与此一致）
        bool is_joint_module = isJointModuleMotor(cfg);
        if (is_joint_module) {
            const bool is_sjd17 = (cfg.vendor_id == kSjd17VendorId &&
                                       cfg.product_code == kSjd17ProductCode);
            const char* tag = is_sjd17 ? "SJD17" : "Xinqi";
            std::cout << "Registering " << tag << " JOINT_MODULE PDO for motor " << i << "..." << std::endl;
            memset(&offsets, 0, sizeof(PDOOffsets));
            offsets.control_word = kPdoOffsetUnset;
            offsets.status_word = kPdoOffsetUnset;
            offsets.actual_position = kPdoOffsetUnset;
            offsets.target_position = kPdoOffsetUnset;
            int ret = 0;

            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x6040, 0x00, domain_out_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register 0x6040" << std::endl; return false; }
            offsets.control_word = ret;
            std::cout << "  ✓ control_word (0x6040): offset=" << ret << std::endl;

            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x607A, 0x00, domain_out_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register 0x607A" << std::endl; return false; }
            offsets.target_position = ret;
            std::cout << "  ✓ target_position (0x607A): offset=" << ret << std::endl;

            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x60FF, 0x00, domain_out_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register 0x60FF" << std::endl; return false; }
            offsets.target_velocity = ret;
            std::cout << "  ✓ target_velocity (0x60FF): offset=" << ret << std::endl;

            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x6071, 0x00, domain_out_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register 0x6071" << std::endl; return false; }
            offsets.target_torque = ret;
            std::cout << "  ✓ target_torque (0x6071): offset=" << ret << std::endl;

            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x6060, 0x00, domain_out_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register 0x6060" << std::endl; return false; }
            offsets.operation_mode = ret;
            std::cout << "  ✓ operation_mode (0x6060): offset=" << ret << std::endl;

            ret = ecrt_slave_config_reg_pdo_entry_pos(slave_configs_[i], 2, 0, 5, domain_out_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register Rx padding (SM2 entry 5)" << std::endl; return false; }
            offsets.padding_rx = ret;
            std::cout << "  ✓ padding_rx (gap): offset=" << ret << std::endl;

            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x6041, 0x00, domain_in_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register 0x6041" << std::endl; return false; }
            offsets.status_word = ret;
            std::cout << "  ✓ status_word (0x6041): offset=" << ret << std::endl;

            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x6064, 0x00, domain_in_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register 0x6064" << std::endl; return false; }
            offsets.actual_position = ret;
            std::cout << "  ✓ actual_position (0x6064): offset=" << ret << std::endl;

            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x606C, 0x00, domain_in_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register 0x606C" << std::endl; return false; }
            offsets.actual_velocity = ret;
            std::cout << "  ✓ actual_velocity (0x606C): offset=" << ret << std::endl;

            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x6077, 0x00, domain_in_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register 0x6077" << std::endl; return false; }
            offsets.actual_torque = ret;
            std::cout << "  ✓ actual_torque (0x6077): offset=" << ret << std::endl;

            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x6061, 0x00, domain_in_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register 0x6061" << std::endl; return false; }
            offsets.operation_mode_display = ret;
            std::cout << "  ✓ operation_mode_display (0x6061): offset=" << ret << std::endl;

            if (is_sjd17) {
                ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x2020, 0x00, domain_in_, nullptr);
                if (ret < 0) { std::cerr << tag << ": Failed to register 0x2020" << std::endl; return false; }
                offsets.sensor_force_2020 = ret;
                std::cout << "  ✓ sensor_force_2020 (0x2020): offset=" << ret << std::endl;

                ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x2021, 0x00, domain_in_, nullptr);
                if (ret < 0) { std::cerr << tag << ": Failed to register 0x2021" << std::endl; return false; }
                offsets.motor_encoder_2021 = ret;
                std::cout << "  ✓ motor_encoder_2021 (0x2021): offset=" << ret << std::endl;

                // SJD17 Tx：entry0..7 = 6041/6064/606C/6077/6061/2020/2021/Gap
                ret = ecrt_slave_config_reg_pdo_entry_pos(slave_configs_[i], 3, 0, 7, domain_in_, nullptr);
                if (ret < 0) { std::cerr << tag << ": Failed to register Tx padding (SM3 entry 7)" << std::endl; return false; }
                offsets.padding_tx = ret;
                std::cout << "  ✓ padding_tx (gap): offset=" << ret << std::endl;
            } else {
                offsets.sensor_force_2020 = 0;
                offsets.motor_encoder_2021 = 0;
                ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x603F, 0x00, domain_in_, nullptr);
                if (ret < 0) { std::cerr << tag << ": Failed to register 0x603F" << std::endl; return false; }
                offsets.error_code = ret;
                std::cout << "  ✓ error_code (0x603F): offset=" << ret << std::endl;

                ret = ecrt_slave_config_reg_pdo_entry_pos(slave_configs_[i], 3, 0, 6, domain_in_, nullptr);
                if (ret < 0) { std::cerr << tag << ": Failed to register Tx padding (SM3 entry 6)" << std::endl; return false; }
                offsets.padding_tx = ret;
                std::cout << "  ✓ padding_tx (gap): offset=" << ret << std::endl;
            }

            std::cout << tag << " JOINT_MODULE " << i << " PDO registration completed." << std::endl;
            std::cout << "  RxPDO offsets: cw=" << offsets.control_word
                      << " tp=" << offsets.target_position << " tv=" << offsets.target_velocity
                      << " tt=" << offsets.target_torque << " om=" << offsets.operation_mode << std::endl;
            std::cout << "  TxPDO offsets: sw=" << offsets.status_word
                      << " ap=" << offsets.actual_position << " av=" << offsets.actual_velocity
                      << " at=" << offsets.actual_torque << " od=" << offsets.operation_mode_display
                      << " ec=" << offsets.error_code
                      << " sf=" << offsets.sensor_force_2020
                      << " me=" << offsets.motor_encoder_2021 << std::endl;
            if (!verifyPdoEvidence(i)) {
                return false;
            }
            continue;
        }

        if (isCoolDriveJmdtMotor(cfg)) {
            const char* tag = "JMDT";
            memset(&offsets, 0, sizeof(PDOOffsets));
            offsets.control_word = kPdoOffsetUnset;
            offsets.status_word = kPdoOffsetUnset;
            offsets.actual_position = kPdoOffsetUnset;
            offsets.target_position = kPdoOffsetUnset;
            int ret = 0;

            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x6040, 0x00, domain_out_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register 0x6040" << std::endl; return false; }
            offsets.control_word = ret;

            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x6060, 0x00, domain_out_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register 0x6060" << std::endl; return false; }
            offsets.operation_mode = ret;

            // 0x5FFE padding（与天机 IgH 一致：按 index 注册）
            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x5FFE, 0x00, domain_out_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register Rx 0x5FFE pad" << std::endl; return false; }
            offsets.padding_rx = ret;

            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x607A, 0x00, domain_out_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register 0x607A" << std::endl; return false; }
            offsets.target_position = ret;

            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x60FF, 0x00, domain_out_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register 0x60FF" << std::endl; return false; }
            offsets.target_velocity = ret;

            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x6071, 0x00, domain_out_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register 0x6071" << std::endl; return false; }
            offsets.target_torque = ret;

            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x6041, 0x00, domain_in_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register 0x6041" << std::endl; return false; }
            offsets.status_word = ret;

            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x6061, 0x00, domain_in_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register 0x6061" << std::endl; return false; }
            offsets.operation_mode_display = ret;

            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x5FFE, 0x00, domain_in_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register Tx 0x5FFE pad" << std::endl; return false; }
            offsets.padding_tx = ret;

            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x6064, 0x00, domain_in_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register 0x6064" << std::endl; return false; }
            offsets.actual_position = ret;

            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x606C, 0x00, domain_in_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register 0x606C" << std::endl; return false; }
            offsets.actual_velocity = ret;

            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x6077, 0x00, domain_in_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register 0x6077" << std::endl; return false; }
            offsets.actual_torque = ret;

            // 0x310B 负载端力矩 → 复用 sensor_force_2020 槽位（产品侧同字段读取）
            ret = ecrt_slave_config_reg_pdo_entry(slave_configs_[i], 0x310B, 0x00, domain_in_, nullptr);
            if (ret < 0) { std::cerr << tag << ": Failed to register 0x310B" << std::endl; return false; }
            offsets.sensor_force_2020 = ret;
            offsets.motor_encoder_2021 = 0;

            if (!verifyPdoEvidence(i)) {
                return false;
            }
            continue;
        }

        std::cerr << "[IgH] unsupported PDO registration: " << cfg.name
                  << " (model=" << cfg.model_id << ")" << std::endl;
        return false;
    }

    std::cout << "[IgH] PDO entries registered for " << motor_count_ << " slaves" << std::endl;
    return true;
}


void EtherCATServo::receiveData()
{
    // 即使未激活也尝试接收（用于激活前的循环），但需要确保master和domain已初始化
    if (!initialized_ || !master_ || !domain_out_ || !domain_in_) return;
    
    ecrt_master_receive(master_);
    ecrt_domain_process(domain_out_);

    if (activated_ && motor_count_ > 0 && slave_configs_[0] && sync_handler_) {
        ec_domain_state_t ds_out{};
        ec_domain_state_t ds_in{};
        ecrt_domain_state(domain_out_, &ds_out);
        ecrt_domain_state(domain_in_, &ds_in);
        ec_slave_config_state_t sc;
        ecrt_slave_config_state(slave_configs_[0], &sc);
        // 参考时钟从站（slave 0）进入 SAFEOP 后即可读参考时钟、启动 DC PLL。
        // 不能要求全部从站 WKC 完整：7 从站时后续从站进 OP 依赖 DC 同步（PLL），
        // 等全 OP 再启动 PLL 会形成死锁（从站卡 PREOP、dc_valid=0、SM2 抖动 → 0xFF51）。
        // 注意 al_state 是从站当前 AL 状态码（SAFEOP=4、OP=8）：进 OP 后仍须保持可读，
        // 否则 `8 & EC_AL_STATE_SAFEOP == 0` 会把 PLL 关停（dc_valid 恒为 0）。
        const bool ref_clock_readable =
            (sc.al_state & (EC_AL_STATE_SAFEOP | EC_AL_STATE_OP));
        sync_handler_->setReferenceClockReady(ref_clock_readable);
    }

    // 启动后短时诊断：观察 WC 是否正常（WC=0 时从站无法进 OP）
    // Job 线程路径禁止 cout
    if (activated_ && motor_count_ > 0 && slave_configs_[0] &&
        !IghMasterRuntime::instance().isJobThreadRunning()) {
        static unsigned int wc_diag_count = 0;
        if (wc_diag_count < 100) {
            ec_domain_state_t ds_out{};
            ec_domain_state_t ds_in{};
            ecrt_domain_state(domain_out_, &ds_out);
            ecrt_domain_state(domain_in_, &ds_in);
            ec_slave_config_state_t sc;
            ecrt_slave_config_state(slave_configs_[0], &sc);
            if (wc_diag_count % 20 == 0) {
                std::cout << "[Diag] cycle=" << wc_diag_count
                          << " WC_out=" << ds_out.working_counter
                          << " WC_in=" << ds_in.working_counter
                          << " wc_out=" << static_cast<int>(ds_out.wc_state)
                          << " wc_in=" << static_cast<int>(ds_in.wc_state)
                          << " AL=0x" << std::hex << static_cast<int>(sc.al_state)
                          << std::dec << " online=" << sc.online
                          << " operational=" << sc.operational;
                if (sc.online) {
                    ec_slave_info_t info{};
                    if (ecrt_master_get_slave(master_, motor_configs_[0].position, &info) == 0) {
                        std::cout << " err=" << static_cast<int>(info.error_flag);
                    }
                }
                std::cout << std::endl;
            }
            wc_diag_count++;
        }
    }
    
    // ========== 位置跟踪调试代码：输出到CSV文件 ==========
    // 启用调试：将 false 改为 true
    static const bool DEBUG_CSV_LOGGING = false;
    static const bool DEBUG_CONSOLE_OUTPUT = false;  // 控制台输出
    
    if (DEBUG_CSV_LOGGING && activated_ && domain_out_ && domain_in_pd_) {
        static auto last_log_time = std::chrono::steady_clock::now();
        static FILE* csv_file = nullptr;
        static bool csv_opened = false;
        static auto start_time = std::chrono::steady_clock::now();
        
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - last_log_time).count();
        
        // 每100ms（0.1秒）记录一次
        if (elapsed >= 100) {
            last_log_time = current_time;
            
            // 打开CSV文件（仅第一次）
            if (!csv_opened) {
                // 生成带时间戳的文件名
                auto now = std::chrono::system_clock::now();
                auto time_t = std::chrono::system_clock::to_time_t(now);
                struct tm* tm_info = localtime(&time_t);
                char filename[100];
                strftime(filename, sizeof(filename), "/tmp/ethercat_position_log_%Y%m%d_%H%M%S.csv", tm_info);
                
                csv_file = fopen(filename, "w");
                if (csv_file) {
                    // 写入CSV表头（单位：度、度/秒、Nm）
                    fprintf(csv_file, "# 位置跟踪调试日志\n");
                    fprintf(csv_file, "# 单位: 位置(度), 速度(度/秒), 力矩(Nm, 关节端输出)\n");
                    fprintf(csv_file,
                            "Time_s,Motor_ID,Target_Position_deg,Actual_Position_deg,Error_deg,"
                            "Idle_Position_deg,Velocity_deg_per_s,Torque_Nm,Is_Moving,Status_Word\n");
                    csv_opened = true;
                    start_time = current_time;
                    std::cout << "\n========================================" << std::endl;
                    std::cout << "✓ 位置跟踪CSV日志已启用" << std::endl;
                    std::cout << "  文件: " << filename << std::endl;
                    std::cout << "  采样间隔: 0.1秒" << std::endl;
                    std::cout << "  单位: 位置(度), 速度(度/秒), 力矩(Nm)" << std::endl;
                    std::cout << "========================================\n" << std::endl;
                }
            }
            
            // 计算相对时间（从记录开始的秒数）
            double relative_time_s = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - start_time).count() / 1000.0;
            
            // ⭐ 已移除 data_mutex_，直接读取数据（调试代码，不影响实时性）
            
            // 控制台输出标题
            if (DEBUG_CONSOLE_OUTPUT) {
                std::cout << "\n========== 位置跟踪 [" << std::fixed << std::setprecision(2) 
                          << relative_time_s << " s] ==========" << std::endl;
            }
            
            // 只处理前7个电机（跳过网关）
            size_t track_count = std::min<size_t>(7, motor_count_);
            for (size_t i = 0; i < track_count; ++i) {
                const auto& cfg = motor_configs_[i];
                // 跳过网关
                if (cfg.vendor_id == kGatewayVendorId && cfg.product_code == kGatewayProductCode) {
                    continue;
                }
                
                const auto& offsets = pdo_offsets_[i];
                
                // 读取实际位置
                int32_t actual_pos = EC_READ_S32(domain_in_pd_ + offsets.actual_position);
                
                // 读取速度和力矩
                int32_t actual_vel = EC_READ_S32(domain_in_pd_ + offsets.actual_velocity);
                int16_t actual_torque = EC_READ_S16(domain_in_pd_ + offsets.actual_torque);
                
                // 读取状态字
                uint16_t status_word = EC_READ_U16(domain_in_pd_ + offsets.status_word);
                
                // 获取目标位置和静止输入位置（使用原子读取）
                int32_t target_pos = target_positions_[i].load(std::memory_order_acquire);
                int32_t idle_pos = idle_input_positions_[i];
                
                // 判断是否在运动中
                bool is_moving = position_override_active_[i];
                
                // 写入CSV文件（转换为实际单位）
                if (csv_file) {
                    // ⭐ 先转换位置（包含偏置量），再计算误差
                    double actual_pos_deg = pulseToDegree(actual_pos, i);
                    double target_pos_deg = pulseToDegree(target_pos, i);
                    double error_deg = target_pos_deg - actual_pos_deg;  // 使用转换后的角度差值
                    double idle_pos_deg = pulseToDegree(idle_pos, i);
                    double actual_vel_deg = pulsePerSecToDegreePerSec(actual_vel, i);
                    double actual_torque_nm = rawTorqueToOutputTorque(actual_torque, i);
                    
                    fprintf(csv_file, "%.6f,%zu,%.4f,%.4f,%.4f,%.4f,%.3f,%.2f,%d,0x%x\n",
                            relative_time_s, i, target_pos_deg, actual_pos_deg, error_deg,
                            idle_pos_deg, actual_vel_deg, actual_torque_nm,
                            is_moving ? 1 : 0, status_word);
                }
                
                // 控制台输出（转换为实际单位）
                if (DEBUG_CONSOLE_OUTPUT) {
                    // ⭐ 转换为实际单位（包含偏置量）
                    double actual_pos_deg = pulseToDegree(actual_pos, i);
                    double target_pos_deg = pulseToDegree(target_pos, i);
                    double error_deg = target_pos_deg - actual_pos_deg;  // 使用转换后的角度差值
                    double actual_vel_deg = pulsePerSecToDegreePerSec(actual_vel, i);
                    double actual_torque_nm = rawTorqueToOutputTorque(actual_torque, i);
                    
                    std::cout << "  电机" << i << ": "
                              << "位置=" << std::setw(8) << std::fixed << std::setprecision(3) << actual_pos_deg << "° "
                              << ", 误差=" << std::setw(7) << std::setprecision(4) << error_deg << "° "
                              << ", 速度=" << std::setw(7) << std::setprecision(3) << actual_vel_deg << "°/s"
                              << ", 力矩=" << std::setw(7) << std::setprecision(2) << actual_torque_nm << "Nm"
                              << (is_moving ? " [运动]" : " [静止]")
                              << std::endl;
                }
            }
            
            if (DEBUG_CONSOLE_OUTPUT) {
                std::cout << "==========================================" << std::endl;
            }
            
            // 刷新文件缓冲区，确保数据写入磁盘
            if (csv_file) {
                fflush(csv_file);
            }
        }
    }
    // ========== 调试代码结束 ==========
    
    // ========== 每周期更新PDO数据缓存 ==========
    // 每周期更新，供ROS发布器使用（seqlock 发布一致性快照）
    if (activated_ && domain_out_ && domain_in_pd_) {
        SeqLockWriter guard(pdo_cache_seq_);
        // 更新所有从站的PDO数据缓存（包括网关）
        for (size_t i = 0; i < motor_count_; ++i) {
            const auto& cfg = motor_configs_[i];
            const auto& offsets = pdo_offsets_[i];
            
            // ⭐ 检查是否是网关：仅通过 VID/PID 识别
            bool is_gateway = (cfg.vendor_id == kGatewayVendorId && cfg.product_code == kGatewayProductCode);
            
            if (is_gateway) {
                // ⭐ 网关处理：读取网关的接收数据并写入内存（即使不安装手，也要持续接收）
                // ⭐ 读取网关发送给主站的所有字段，确保网关正常工作
                
                GatewayData& gw_data = gateway_data_[i];
                
                // CAN/CANFD接收：读取ID、长度和数据字段并保存到内存
                if (offsets.can_rx_id != 0) {
                    gw_data.can_rx_id = EC_READ_U32(domain_in_pd_ + offsets.can_rx_id);
                }
                if (offsets.can_rx_length != 0) {
                    gw_data.can_rx_length = EC_READ_U8(domain_in_pd_ + offsets.can_rx_length);
                }
                if (offsets.can_rx_data != 0) {
                    // 读取CAN RX数据（64字节）并保存到内存
                    memcpy(gw_data.can_rx_data, domain_in_pd_ + offsets.can_rx_data, 64);
                }
                
                // RS485_1发送：读取长度和数据字段并保存到内存
                if (offsets.rs485_1_tx_length != 0) {
                    gw_data.rs485_1_tx_length = EC_READ_U8(domain_in_pd_ + offsets.rs485_1_tx_length);
                }
                if (offsets.rs485_1_tx_data != 0) {
                    // 读取RS485_1 TX数据（64字节）并保存到内存
                    memcpy(gw_data.rs485_1_tx_data, domain_in_pd_ + offsets.rs485_1_tx_data, 64);
                }
                
                // RS485_2发送：读取长度和数据字段并保存到内存
                if (offsets.rs485_2_tx_length != 0) {
                    gw_data.rs485_2_tx_length = EC_READ_U8(domain_in_pd_ + offsets.rs485_2_tx_length);
                }
                if (offsets.rs485_2_tx_data != 0) {
                    // 读取RS485_2 TX数据（64字节）并保存到内存
                    memcpy(gw_data.rs485_2_tx_data, domain_in_pd_ + offsets.rs485_2_tx_data, 64);
                }
                
                // ⭐ 网关不需要缓存电机相关的数据，跳过
                continue;
            }
            
            // 电机处理：读取并缓存所有PDO数据
            // 注意：这里只是内存拷贝，速度很快
            last_status_words_[i] = EC_READ_U16(domain_in_pd_ + offsets.status_word);
            last_operation_mode_displays_[i] = EC_READ_S8(domain_in_pd_ + offsets.operation_mode_display);
            if (offsets.error_code != 0) {
                last_error_codes_[i] = EC_READ_U16(domain_in_pd_ + offsets.error_code);
            } else {
                last_error_codes_[i] = 0;
            }
            last_actual_positions_[i] = EC_READ_S32(domain_in_pd_ + offsets.actual_position);
            last_actual_velocities_[i] = EC_READ_S32(domain_in_pd_ + offsets.actual_velocity);
            last_actual_torques_[i] = EC_READ_S16(domain_in_pd_ + offsets.actual_torque);
            if (offsets.sensor_force_2020 != 0) {
                last_sensor_force_2020_[i] = EC_READ_S32(domain_in_pd_ + offsets.sensor_force_2020);
            } else {
                last_sensor_force_2020_[i] = 0;
            }
            if (offsets.motor_encoder_2021 != 0) {
                last_motor_encoder_2021_[i] = EC_READ_S32(domain_in_pd_ + offsets.motor_encoder_2021);
            } else {
                last_motor_encoder_2021_[i] = 0;
            }
        }
    }
    // ========== PDO数据缓存更新结束 ==========
    
    // 调试：每1000次打印一次PDO数据（在非RT上下文执行）
    // 注意：std::cout 不适合在RT循环中使用，已禁用以减少延迟
    /*
    static int debug_counter = 0;
    if (++debug_counter % 1000 == 0 && motor_count_ > 0) {
        std::cout << "\n=== PDO数据调试 (第" << debug_counter << "次) ===" << std::endl;
        for (size_t i = 0; i < motor_count_; ++i) {
            const auto& offsets = pdo_offsets_[i];
            
            // 打印所有偏移量
            std::cout << "Motor " << i << " PDO偏移: "
                      << "status=" << offsets.status_word 
                      << " mode=" << offsets.operation_mode_display
                      << " pos=" << offsets.actual_position
                      << " vel=" << offsets.actual_velocity
                      << " torque=" << offsets.actual_torque << std::endl;
            
            // 打印原始数据
            uint16_t status = EC_READ_U16(domain_in_pd_ + offsets.status_word);
            int8_t mode = EC_READ_S8(domain_in_pd_ + offsets.operation_mode_display);
            int32_t pos = EC_READ_S32(domain_in_pd_ + offsets.actual_position);
            int32_t vel = EC_READ_S32(domain_in_pd_ + offsets.actual_velocity);
            int16_t torque = EC_READ_S16(domain_in_pd_ + offsets.actual_torque);
            
            std::cout << "Motor " << i << " 数据: "
                      << "status=0x" << std::hex << status << std::dec
                      << " mode=" << (int)mode
                      << " pos=" << pos
                      << " vel=" << vel
                      << " torque=" << torque << std::endl;
        }
        std::cout << "=== 使用 ethercat 命令验证: sudo ethercat upload -p 0 0x6041 0 ===" << std::endl;
    }
    */
}

void EtherCATServo::sendData()
{
    // 即使未激活也发送数据（保持数据不变），用于激活前的循环
    // 但需要确保双域过程数据已初始化
    if (!initialized_ || !domain_out_pd_ || !domain_in_pd_) return;

    if (runtime_logger_) {
        runtime_logger_->flushPendingToBuffer();
    }

    g_send_data_wc = 0;
    g_send_data_wc_state = 0;
    if (activated_ && domain_out_ && domain_in_) {
        ec_domain_state_t ds_out{};
        ec_domain_state_t ds_in{};
        ecrt_domain_state(domain_out_, &ds_out);
        ecrt_domain_state(domain_in_, &ds_in);
        const bool ok =
            ds_out.working_counter > 0 && ds_in.working_counter > 0 &&
            ds_out.wc_state == EC_WC_COMPLETE && ds_in.wc_state == EC_WC_COMPLETE;
        g_send_data_wc = ok ? ds_out.working_counter : 0;
        g_send_data_wc_state = ok ? EC_WC_COMPLETE : ds_out.wc_state;
    }
    
    // 处理所有从站（包括网关）
    for (size_t i = 0; i < motor_count_; ++i) {
        const auto& cfg = motor_configs_[i];
        const auto& offsets = pdo_offsets_[i];
        
        // ⭐ 检查是否是网关：仅通过 VID/PID 识别
        bool is_gateway = (cfg.vendor_id == kGatewayVendorId && cfg.product_code == kGatewayProductCode);
        
        if (is_gateway) {
            // ⭐ 网关处理：写入所有发送PDO字段为0，确保网关正常工作
            // 即使不安装手，也要持续发送数据，只是数据长度和内容都是0
            
            // CAN/CANFD发送：写入ID、长度和数据字段为0
            if (offsets.can_tx_id != 0) {
                EC_WRITE_U32(domain_out_pd_ + offsets.can_tx_id, 0);  // CAN TX ID = 0
            }
            if (offsets.can_tx_length != 0) {
                EC_WRITE_U8(domain_out_pd_ + offsets.can_tx_length, 0);  // CAN/CANFD发送长度 = 0
            }
            if (offsets.can_tx_data != 0) {
                // 清零CAN TX数据（64字节）
                memset(domain_out_pd_ + offsets.can_tx_data, 0, 64);
            }
            
            // RS485_1接收：写入长度和数据字段为0
            if (offsets.rs485_1_rx_length != 0) {
                EC_WRITE_U8(domain_out_pd_ + offsets.rs485_1_rx_length, 0);  // RS485_1接收长度 = 0（主站发送给网关）
            }
            if (offsets.rs485_1_rx_data != 0) {
                // 清零RS485_1 RX数据（64字节）
                memset(domain_out_pd_ + offsets.rs485_1_rx_data, 0, 64);
            }
            
            // RS485_2接收：写入长度和数据字段为0
            if (offsets.rs485_2_rx_length != 0) {
                EC_WRITE_U8(domain_out_pd_ + offsets.rs485_2_rx_length, 0);  // RS485_2接收长度 = 0（主站发送给网关）
            }
            if (offsets.rs485_2_rx_data != 0) {
                // 清零RS485_2 RX数据（64字节）
                memset(domain_out_pd_ + offsets.rs485_2_rx_data, 0, 64);
            }
            
            // ⭐ 注意：接收长度字段（网关发送给主站）在receiveData()中读取，这里不需要处理
            continue;  // 跳过后续的电机处理
        }
        
        // 电机处理（前7个电机）

        // ⭐ 从 pending_commands_ 读取待处理命令并应用到目标值
        auto& pending = pending_commands_[i];
        if (pending.position_valid.load(std::memory_order_acquire)) {
            target_positions_[i].store(pending.position.load(std::memory_order_acquire), std::memory_order_relaxed);
            pending.position_valid.store(false, std::memory_order_release);
        }
        if (pending.velocity_valid.load(std::memory_order_acquire)) {
            target_velocities_[i].store(pending.velocity.load(std::memory_order_acquire), std::memory_order_relaxed);
            pending.velocity_valid.store(false, std::memory_order_release);
        }
        if (pending.torque_valid.load(std::memory_order_acquire)) {
            target_torques_[i].store(pending.torque.load(std::memory_order_acquire), std::memory_order_relaxed);
            pending.torque_valid.store(false, std::memory_order_release);
        }

        // 1. 通用部分：控制字、操作模式等
        sendDataCommon(i, offsets);
        
        // 2. 每周期写满 RxPDO 过程数据（从站进入 OP 的必要条件）
        sendDataCSP(i, offsets);
        sendDataCSV(i, offsets);
        sendDataCST(i, offsets);
    }
    
    ecrt_domain_queue(domain_out_);
    ecrt_master_send(master_);
}

void EtherCATServo::fillRuntimeEventTargets(RuntimeLogEvent& ev, size_t motor_id) const
{
    if (motor_id >= motor_count_) {
        return;
    }
    ev.target_position = target_positions_[motor_id].load(std::memory_order_acquire);
    ev.target_velocity = target_velocities_[motor_id].load(std::memory_order_acquire);
    ev.target_torque = target_torques_[motor_id].load(std::memory_order_acquire);
}

void EtherCATServo::maybeLogTargetSetpoint(size_t motor_id, uint16_t status_word,
                                           uint16_t cia402_state, uint16_t control_word)
{
    if (!runtime_logger_ || !activated_ || motor_id >= motor_count_) {
        return;
    }

    constexpr uint16_t kTargetLogIntervalCycles = 100;  // 10Hz @ 1kHz
    if (++target_setpoint_log_counters_[motor_id] < kTargetLogIntervalCycles) {
        return;
    }
    target_setpoint_log_counters_[motor_id] = 0;

    const int32_t target_pos = target_positions_[motor_id].load(std::memory_order_acquire);
    const int32_t target_vel = target_velocities_[motor_id].load(std::memory_order_acquire);
    const int16_t target_tor = target_torques_[motor_id].load(std::memory_order_acquire);

    const bool in_motion = position_override_active_[motor_id] || velocity_override_active_[motor_id];
    const bool changed =
        target_pos != last_logged_target_positions_[motor_id] ||
        target_vel != last_logged_target_velocities_[motor_id] ||
        target_tor != last_logged_target_torques_[motor_id];

    if (!in_motion && !changed) {
        return;
    }

    RuntimeLogEvent ev{};
    ev.timestamp_ns = getMonotonicTimeNs();
    ev.motor_id = static_cast<uint8_t>(motor_id);
    ev.type = RuntimeLogEventType::TARGET_SETPOINT;
    ev.status_word = status_word;
    ev.cia402_state = cia402_state;
    ev.control_word = control_word;
    ev.operation_mode = static_cast<int8_t>(current_modes_[motor_id]);
    ev.target_position = target_pos;
    ev.target_velocity = target_vel;
    ev.target_torque = target_tor;
    runtime_logger_->pushRtEvent(ev);

    last_logged_target_positions_[motor_id] = target_pos;
    last_logged_target_velocities_[motor_id] = target_vel;
    last_logged_target_torques_[motor_id] = target_tor;
}

// ========== 通用部分：控制字、操作模式 ==========

void EtherCATServo::sendDataCommon(size_t i, const PDOOffsets& offsets)
{
    // CiA402 enable/disable FSM on Job TX. STEP_DELAY_CYCLES is in bus cycles (@1ms default
    // ≈ 10ms/step). Transitions require wc_ok; fault bit (0x08) runs FAULT_RESET edge phases.
    // Safe-output clears desired_enable_ / FSM so latched disable cannot resume mid-step.
        const auto& enable_sequence = kCia402EnableSequence;
        const auto& disable_sequence = kCia402DisableSequence;
        const uint16_t STEP_DELAY_CYCLES = 10; // 10 bus cycles per step
        constexpr int FAULT_DEBOUNCE_CYCLES = 5;        // confirm Fault
        constexpr int ENABLE_LOSS_DEBOUNCE_CYCLES = 50; // confirm unexpected drop from OP

        const bool wc_ok = (g_send_data_wc_state == EC_WC_COMPLETE);

        // 读取状态字（故障检测 / 使能监控）
        uint16_t sw = EC_READ_U16(domain_in_pd_ + offsets.status_word);
        bool has_fault = isCiA402Fault(sw);
        bool is_operation_enabled = isCiA402OperationEnabled(sw);
        const CIA402State cia_state = getState(sw);
        const uint16_t cia_state_value = static_cast<uint16_t>(cia_state);

        if (runtime_logger_) {
            const bool status_changed = (sw != last_logged_status_words_[i]);
            const bool state_changed = (cia_state_value != last_logged_cia402_states_[i]);
            if (status_changed || state_changed) {
                RuntimeLogEvent ev{};
                ev.timestamp_ns = getMonotonicTimeNs();
                ev.motor_id = static_cast<uint8_t>(i);
                ev.type = RuntimeLogEventType::CIA402_STATE;
                ev.status_word = sw;
                ev.cia402_state = cia_state_value;
                ev.control_word = control_word_states_[i].load(std::memory_order_acquire);
                ev.operation_mode = static_cast<int8_t>(current_modes_[i]);
                fillRuntimeEventTargets(ev, i);
                runtime_logger_->pushRtEvent(ev);
                last_logged_status_words_[i] = sw;
                last_logged_cia402_states_[i] = cia_state_value;

                if (has_fault) {
                    RuntimeLogEvent fault_ev = ev;
                    fault_ev.type = RuntimeLogEventType::ERROR_CODE;
                    fault_ev.error_code = 0;
                    std::snprintf(fault_ev.detail, sizeof(fault_ev.detail), "fault_detected");
                    runtime_logger_->pushRtEvent(fault_ev);
                }
            }
        }

        // ⭐ Fault 检测：去抖后立即启动故障复位（不等待 1s 监控周期）
        if (wc_ok && has_fault && desired_enable_[i] && enable_requested_[i]
            && !enable_fsm_active_[i] && fault_reset_phase_[i] == 0
            && fault_reset_counter_[i] == 0) {
            fault_persist_counter_[i]++;
            if (fault_persist_counter_[i] >= FAULT_DEBOUNCE_CYCLES) {
                fault_persist_counter_[i] = 0;
                enable_fsm_active_[i] = true;
                enable_requested_[i] = false;
            }
        } else if (!has_fault) {
            fault_persist_counter_[i] = 0;
        }

        // ⭐ 使能监控：仅处理「无 Fault 但意外退出 OP」
        if (wc_ok && desired_enable_[i] && enable_requested_[i] && !enable_fsm_active_[i]
            && fault_reset_counter_[i] == 0 && !has_fault) {
            if (!is_operation_enabled) {
                enable_monitor_loss_counter_[i]++;
            } else {
                enable_monitor_loss_counter_[i] = 0;
            }
            if (enable_monitor_loss_counter_[i] >= ENABLE_LOSS_DEBOUNCE_CYCLES) {
                enable_monitor_loss_counter_[i] = 0;
                enable_fsm_active_[i] = true;
                enable_fsm_step_[i] = 0;
                enable_fsm_wait_[i] = 0;
                control_word_states_[i].store(CONTROL_WORD_SWITCH_ON, std::memory_order_release);
                enable_requested_[i] = false;
            }
        } else if (is_operation_enabled || has_fault) {
            enable_monitor_loss_counter_[i] = 0;
        }

        // ⭐ 故障复位后自动恢复使能
        if (fault_reset_counter_[i] > 0) {
            fault_reset_counter_[i]--;
            if (fault_reset_counter_[i] == 0 && desired_enable_[i]) {
                // 故障复位完成，自动重新进入使能序列
                // 确保状态机保持活跃，从0x06开始重新使能
                enable_fsm_active_[i] = true;
                enable_fsm_step_[i] = 0;
                enable_fsm_wait_[i] = 0;
                control_word_states_[i].store(CONTROL_WORD_SWITCH_ON, std::memory_order_release); // 从0x06开始
                // 注意：不修改 enable_requested_，保持 desired_enable_ 和 enable_requested_ 不一致
                // 这样状态机会自动检测到需要重新使能
            }
        }

        // ⭐ 如果 fault_reset_phase_ 不为0但已经没有故障或不需要使能，重置相位
        if (fault_reset_phase_[i] != 0 && (!has_fault || !desired_enable_[i])) {
            fault_reset_phase_[i] = 0;
        }
        
        // 如有外部请求，启动/维持状态机
        if (desired_enable_[i] != enable_requested_[i] || enable_fsm_active_[i]) {
            enable_fsm_active_[i] = true;
            // 等待计数减至0后推进一步
            if (enable_fsm_wait_[i] > 0) {
                enable_fsm_wait_[i]--;
            } else {
                if (has_fault && desired_enable_[i]) {
                    // ⭐ 故障复位状态机：产生正确的 0→1→0 跳变（CiA 402要求bit 7上升沿触发故障复位）
                    if (fault_reset_phase_[i] == 0) {
                        // Phase 0: 发送 Fault Reset 控制字（profile 化：新奇/三木禾 0x86，天机 0x80）
                        control_word_states_[i].store(
                          fault_reset_cw_[i], std::memory_order_release);
                        fault_reset_phase_[i] = 1;
                        enable_fsm_wait_[i] = STEP_DELAY_CYCLES;  // 保持0x80持续10ms
                        enable_fsm_step_[i] = 0;
                        enable_requested_[i] = false;
                    } else if (fault_reset_phase_[i] == 1) {
                        // Phase 1: 继续发送0x80，等待STEP_DELAY_CYCLES结束
                        control_word_states_[i].store(
                          fault_reset_cw_[i], std::memory_order_release);
                        if (enable_fsm_wait_[i] > 0) {
                            enable_fsm_wait_[i]--;
                        } else {
                            // 切换到0x06（清除bit 7），完成上升沿→下降沿的完整跳变
                            fault_reset_phase_[i] = 2;
                            enable_fsm_wait_[i] = STEP_DELAY_CYCLES;
                        }
                    } else if (fault_reset_phase_[i] == 2) {
                        // Phase 2: 发送 0x06 (Shutdown) - 清除 bit 7 = 0
                        control_word_states_[i].store(CONTROL_WORD_SWITCH_ON, std::memory_order_release);
                        if (enable_fsm_wait_[i] > 0) {
                            enable_fsm_wait_[i]--;
                        } else {
                            // Phase 2结束：检查故障是否已清除
                            uint16_t sw_now = EC_READ_U16(domain_in_pd_ + offsets.status_word);
                            if ((sw_now & 0x08) == 0) {
                                // ✓ 故障已清除！设置fault_reset_counter_触发自动使能（通过line 1854的机制）
                                fault_reset_counter_[i] = 1;  // 下一周期减到0，触发使能
                                fault_reset_phase_[i] = 0;
                            } else {
                                // 故障未清除，重新尝试（回到Phase 0）
                                fault_reset_phase_[i] = 0;
                                enable_fsm_wait_[i] = 0;
                            }
                        }
                    }
                } else {
                    if (desired_enable_[i]) {
                        // 使能方向
                        if (enable_fsm_step_[i] < 4) {
                            control_word_states_[i].store(enable_sequence[enable_fsm_step_[i]], std::memory_order_release);
                            enable_fsm_step_[i]++;
                            enable_fsm_wait_[i] = STEP_DELAY_CYCLES;
                        } else {
                            enable_requested_[i] = true;
                            enable_fsm_active_[i] = false;
                            enable_fsm_step_[i] = 0;
                            enable_fsm_wait_[i] = 0;
                            fault_reset_counter_[i] = 0; // 清除故障复位计数器
                        }
                    } else {
                        // 失能方向
                        if (enable_fsm_step_[i] < 4) {
                            control_word_states_[i].store(disable_sequence[enable_fsm_step_[i]], std::memory_order_release);
                            enable_fsm_step_[i]++;
                            enable_fsm_wait_[i] = STEP_DELAY_CYCLES;
                        } else {
                            enable_requested_[i] = false;
                            enable_fsm_active_[i] = false;
                            enable_fsm_step_[i] = 0;
                            enable_fsm_wait_[i] = 0;
                            fault_reset_counter_[i] = 0; // 清除故障复位计数器
                        }
                    }
                }
            }
        }
        
    // 写入控制字 (0x6040)
        uint16_t control_word;
        
        if (!activated_) {
            // 未激活时，保持0x06 (Shutdown)
            control_word = CONTROL_WORD_SWITCH_ON;
        } else {
            // 已激活时，直接使用 setEnable 设置的 control_word_states_ 值（原子读取）
            control_word = control_word_states_[i].load(std::memory_order_acquire);
        }
        
    // 控制字写入逻辑：优先使用待写入缓存
        uint16_t control_word_to_write = control_word;
        
        if (control_word_write_pending_[i] && pending_control_words_[i] != control_word_states_[i].load(std::memory_order_acquire)) {
            control_word_to_write = pending_control_words_[i];
            control_word_states_[i].store(pending_control_words_[i], std::memory_order_release);
        }
        
    // 写入控制字到PDO
        EC_WRITE_U16(domain_out_pd_ + offsets.control_word, control_word_to_write);
        
    // 更新最后写入的控制字
    last_written_control_words_[i] = control_word_to_write;

    if (runtime_logger_ && control_word_to_write != last_logged_control_words_[i]) {
            RuntimeLogEvent ev{};
            ev.timestamp_ns = getMonotonicTimeNs();
            ev.motor_id = static_cast<uint8_t>(i);
            ev.type = RuntimeLogEventType::CONTROL_WORD;
            ev.status_word = sw;
            ev.cia402_state = cia_state_value;
            ev.control_word = control_word_to_write;
            ev.operation_mode = static_cast<int8_t>(current_modes_[i]);
            std::snprintf(ev.detail, sizeof(ev.detail), "control_word_write");
            fillRuntimeEventTargets(ev, i);
            runtime_logger_->pushRtEvent(ev);
            last_logged_control_words_[i] = control_word_to_write;
        }
        
    // 清除 pending 标记
        if (control_word_write_pending_[i] && control_word_to_write == pending_control_words_[i]) {
            control_word_write_pending_[i] = false;
            control_word_write_attempts_[i] = 0;
        }
        
    // 写入操作模式 (0x6060)
        if (offsets.operation_mode != 0) {
            int8_t mode_value = static_cast<int8_t>(current_modes_[i]);
            EC_WRITE_S8(domain_out_pd_ + offsets.operation_mode, mode_value);
        }
        if (offsets.padding_rx != 0) {
            EC_WRITE_U8(domain_out_pd_ + offsets.padding_rx, 0);
        }

        maybeLogTargetSetpoint(i, sw, cia_state_value, control_word_to_write);
}

// ========== CSP 模式特定处理 ==========

void EtherCATServo::sendDataCSP(size_t i, const PDOOffsets& offsets)
{
    // CSP模式：
    // 1. 静止时：使用idle_input_positions_[i]填充buffer并保持位置
    // 2. 运动时：轨迹规划器通过setTargetPosition设置目标位置
    // 使用可配置大小的平均滤波（默认3点）
    
    // // ⭐ 关键调试：输出前5次调用的状态
    // static std::map<size_t, int> call_count;
    // if (call_count[i]++ < 5) {
    //     std::cout << "[sendDataCSP] 电机" << i << " 第" << call_count[i] << "次: "
    //               << "activated=" << activated_ 
    //               << ", override=" << position_override_active_[i]
    //               << ", target_pos=" << target_positions_[i]
    //               << ", idle_pos=" << idle_input_positions_[i] << std::endl;
    // }
    
    if (activated_ && !position_override_active_[i]) {
        // ⭐ 停止状态：直接使用idle_input_positions_作为目标位置，不经过滤波器
        // 这样可以立即响应idle位置的变化，避免滤波延迟
        target_positions_[i].store(idle_input_positions_[i], std::memory_order_relaxed);
    }
    
    // 写入目标位置（无论是否激活都要写入）- 使用原子读取
    int32_t target_pos = target_positions_[i].load(std::memory_order_acquire);
    EC_WRITE_S32(domain_out_pd_ + offsets.target_position, target_pos);
}

// ========== CSV 模式特定处理 ==========

void EtherCATServo::sendDataCSV(size_t i, const PDOOffsets& offsets)
{
    // CSV模式：
    // 1. 静止时：使用idle_input_velocities_[i]（通常为0）
    // 2. 运动时：轨迹规划器通过setTargetVelocity设置目标速度
    
    if (activated_ && !velocity_override_active_[i]) {
        // ⭐ 停止状态：直接使用idle速度（通常为0）
        target_velocities_[i].store(idle_input_velocities_[i], std::memory_order_relaxed);
    }
    
    // 写入目标速度 (0x60FF) - 使用原子读取
    if (offsets.target_velocity != 0) {
        int32_t target_vel = target_velocities_[i].load(std::memory_order_acquire);
        EC_WRITE_S32(domain_out_pd_ + offsets.target_velocity, target_vel);
    }
}
        
// ========== CST 模式特定处理 ==========

void EtherCATServo::sendDataCST(size_t i, const PDOOffsets& offsets)
{
        // CST模式：写入目标力矩 (0x6071) - 使用原子读取
        if (offsets.target_torque != 0) {
            int16_t target_torque = target_torques_[i].load(std::memory_order_acquire);
            EC_WRITE_S16(domain_out_pd_ + offsets.target_torque, target_torque);
        }
        
        // 写入力矩偏移 (0x60B2) - 可选，暂时设置为 0
        if (offsets.digital_outputs != 0) {
            EC_WRITE_S16(domain_out_pd_ + offsets.digital_outputs, 0);
        }
}


CIA402State EtherCATServo::getState(uint16_t status_word) const
{
    return decodeCia402State(status_word);
}

bool EtherCATServo::setEnable(uint8_t motor_id, bool enable)
{
    if (!activated_) {
        return false;
    }
    if (enable && (commFault() || safeOutputRequired() || !motionReenableAllowed())) {
        return false;
    }
    if (enable && !startup_evidence_passed_) {
        std::cerr << "[IgH] setEnable blocked: startup evidence gate not passed"
                  << " (observation-only)" << std::endl;
        return false;
    }
    // Non-RT writer; Job TX path reads desired_enable_ / FSM without mutex.
    if (motor_id == 0xFF) {
        // Broadcast: all motors, skip gateway VID/PID.
        bool any_change = false;
        for (size_t i = 0; i < motor_count_; ++i) {
            const auto& cfg = motor_configs_[i];
            if (cfg.vendor_id == kGatewayVendorId && cfg.product_code == kGatewayProductCode) {
                continue;
            }
            if (desired_enable_[i] != enable) {
                any_change = true;
                break;
            }
        }
        // 幂等：desired_enable 未变化时直接返回，勿重置使能 FSM。
        // Master::cycle 每拍都会调用 setEnable，无条件重置会让使能序列永远停在 0x06。
        if (!any_change) {
            return true;
        }
        for (size_t i = 0; i < motor_count_; ++i) {
            const auto& cfg = motor_configs_[i];
            if (cfg.vendor_id == kGatewayVendorId && cfg.product_code == kGatewayProductCode) {
                continue;
            }
            desired_enable_[i] = enable;
            enable_fsm_active_[i] = true;
            enable_fsm_step_[i] = 0;
            enable_fsm_wait_[i] = 0;
        }
    } else if (motor_id < motor_count_) {
        // 幂等：同上；仅在使能状态变化时启动/重置 FSM
        if (desired_enable_[motor_id] == enable) {
            return true;
        }
        desired_enable_[motor_id] = enable;
        enable_fsm_active_[motor_id] = true;
        enable_fsm_step_[motor_id] = 0;
        enable_fsm_wait_[motor_id] = 0;
    } else {
        return false;
    }

    if (runtime_logger_) {
        RuntimeLogEvent ev{};
        ev.timestamp_ns = getMonotonicTimeNs();
        ev.motor_id = motor_id;
        ev.type = RuntimeLogEventType::SERVO_COMMAND;
        std::snprintf(ev.detail, sizeof(ev.detail), "set_enable enable=%d", enable ? 1 : 0);
        const size_t log_motor = (motor_id == 0xFF || motor_id >= motor_count_) ? 0 : motor_id;
        fillRuntimeEventTargets(ev, log_motor);
        runtime_logger_->queueCommand(ev);
    }
    return true;
}

bool EtherCATServo::requestFaultReset(uint8_t motor_id, bool allow_without_fault) noexcept
{
    // 门禁：Job OP + safe-output 已闩 + 无通信闩锁；所选轴须已失能。
    // 默认要求至少一轴 Fault(bit3) 或缓存 error_code≠0；启动清 0xFF51 可 allow_without_fault。
    if (!initialized_ || !activated_ || !isJobThreadRunning() ||
        !safeOutputRequired() || commFault() || !areAllSlavesInOP() ||
        (motor_id != 0xFFU && motor_id >= motor_count_))
    {
        return false;
    }

    bool selected_fault_present = false;
    for (size_t i = 0; i < motor_count_; ++i) {
        if (isGateway(motor_configs_[i]) ||
            (motor_id != 0xFFU && motor_id != static_cast<uint8_t>(i)))
        {
            continue;
        }
        const uint16_t sw =
          (i < last_status_words_.size()) ? last_status_words_[i] : 0U;
        if (isCiA402OperationEnabled(sw)) {
            return false;
        }
        const uint16_t ec =
          (i < last_error_codes_.size()) ? last_error_codes_[i] : 0U;
        selected_fault_present = selected_fault_present ||
          isCiA402Fault(sw) || ec != 0U;
    }
    if (!selected_fault_present && !allow_without_fault) {
        return false;
    }

    explicit_fault_reset_axis_.store(
        motor_id == 0xFFU ? 0x00FFU : static_cast<uint16_t>(motor_id),
        std::memory_order_release);
    // 二十个总线周期（2 ms 周期时约 40 ms，对齐参考实现）内写 Fault Reset 控制字。
    explicit_fault_reset_cycles_.store(20U, std::memory_order_release);
    return true;
}

bool EtherCATServo::setOperationMode(uint8_t motor_id, OperationMode mode)
{
    auto disarmLeavingMode = [this](size_t i, OperationMode previous, OperationMode next) {
        if (previous == OperationMode::CYCLIC_SYNC_TORQUE &&
            next != OperationMode::CYCLIC_SYNC_TORQUE)
        {
            disarmCommandFreshness(torque_cmd_freshness_[i]);
        }
        if (previous == OperationMode::CYCLIC_SYNC_VELOCITY &&
            next != OperationMode::CYCLIC_SYNC_VELOCITY)
        {
            disarmCommandFreshness(velocity_cmd_freshness_[i]);
        }
    };

    // 幂等：已在目标模式则跳过（避免 Master::cycle 每拍 sleep/日志）
    if (motor_id == 0xFF) {
        bool all_same = true;
        for (size_t i = 0; i < motor_count_; ++i) {
            const auto & cfg = motor_configs_[i];
            if (cfg.vendor_id == kGatewayVendorId && cfg.product_code == kGatewayProductCode) {
                continue;
            }
            if (i >= current_modes_.size() || current_modes_[i] != mode) {
                all_same = false;
                break;
            }
        }
        if (all_same) {
            return true;
        }
    } else if (motor_id < current_modes_.size() && current_modes_[motor_id] == mode) {
        return true;
    }

    // 模式切换前，先检查并失能所有电机
    bool was_enabled = false;
    if (activated_) {
        for (size_t i = 0; i < motor_count_; ++i) {
            if (enable_requested_[i]) {
                was_enabled = true;
                break;
            }
        }
        
        if (was_enabled) {
            std::cout << "Disabling all motors before mode change..." << std::endl;
            setEnable(0xFF, false);
            // 等待失能 FSM 完成（至少数个 EtherCAT 周期）
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
    }
    
    // ⭐⭐⭐ 关键修复：如果切换到CSP模式，先初始化位置参数，再设置模式（避免突变）
    if (activated_ && mode == OperationMode::CYCLIC_SYNC_POSITION) {
        std::cout << "\n=== 切换到CSP模式：先初始化位置参数（避免突变） ===" << std::endl;
        std::cout << "从last_actual_positions缓存读取当前位置并更新所有位置参数" << std::endl;
        
        // ⭐ 确定需要初始化的电机范围（在锁外计算，减少锁持有时间）
        size_t process_count = (motor_id == 0xFF) ? motor_count_ : (motor_id + 1);
        size_t start_idx = (motor_id == 0xFF) ? 0 : motor_id;
        
        // ⭐ 已移除 data_mutex_，直接读取缓存（低频更新，线程安全）
        std::vector<int32_t> positions;
        positions.reserve(process_count - start_idx);
        for (size_t motor_idx = start_idx; motor_idx < process_count; ++motor_idx) {
            const auto& cfg = motor_configs_[motor_idx];
            if (cfg.vendor_id == kGatewayVendorId && cfg.product_code == kGatewayProductCode) {
                continue;
            }
            positions.push_back(last_actual_positions_[motor_idx]);
        }
        
        // ⭐ 已移除 data_mutex_，使用原子操作
        size_t pos_idx = 0;
        for (size_t motor_idx = start_idx; motor_idx < process_count; ++motor_idx) {
            // 跳过网关
            const auto& cfg = motor_configs_[motor_idx];
            if (cfg.vendor_id == kGatewayVendorId && cfg.product_code == kGatewayProductCode) {
                continue;
            }
            
            int32_t final_pos = positions[pos_idx++];
            
            // ⭐ 更新所有位置相关参数（直接更新，无需buffer）
            idle_input_positions_[motor_idx] = final_pos;
            // ⭐ 使用原子操作更新目标位置
            target_positions_[motor_idx].store(final_pos, std::memory_order_release);
            
            // ⭐⭐⭐ 关键：位置参数初始化完成后，再设置模式（避免在初始化前发送错误位置）
            disarmLeavingMode(motor_idx, current_modes_[motor_idx], mode);
            current_modes_[motor_idx] = mode;
            // 模式切换时，重置控制字状态
            if (!enable_requested_[motor_idx]) {
                control_word_states_[motor_idx].store(CONTROL_WORD_SWITCH_ON, std::memory_order_release);
                pending_control_words_[motor_idx] = CONTROL_WORD_SWITCH_ON;
            }
        }
        
        // ⭐ 打印信息移到锁外（避免阻塞）
        std::cout << "✓ CSP模式位置参数初始化完成，模式已设置" << std::endl;
        std::cout << "✓ CSP模式已稳定，现在可以使能电机\n" << std::endl;
    } else if (activated_ && mode == OperationMode::CYCLIC_SYNC_VELOCITY) {
        // ⭐⭐⭐ CSV模式：初始化速度为0，再设置模式（避免突变）
        std::cout << "\n=== 切换到CSV模式：先初始化速度参数（避免突变） ===" << std::endl;
        std::cout << "设置所有速度为0" << std::endl;
        
        // ⭐ 确定需要初始化的电机范围
        size_t process_count = (motor_id == 0xFF) ? motor_count_ : (motor_id + 1);
        size_t start_idx = (motor_id == 0xFF) ? 0 : motor_id;
        
        // ⭐ 已移除 data_mutex_，使用原子操作初始化速度
        for (size_t motor_idx = start_idx; motor_idx < process_count; ++motor_idx) {
            // 跳过网关
            const auto& cfg = motor_configs_[motor_idx];
            if (cfg.vendor_id == kGatewayVendorId && cfg.product_code == kGatewayProductCode) {
                continue;
            }
            
            // ⭐ 初始化所有速度为0
            idle_input_velocities_[motor_idx] = 0;
            // ⭐ 使用原子操作更新目标速度
            target_velocities_[motor_idx].store(0, std::memory_order_release);
            velocity_override_active_[motor_idx] = false;
            
            // ⭐⭐⭐ 关键：速度参数初始化完成后，再设置模式
            disarmLeavingMode(motor_idx, current_modes_[motor_idx], mode);
            current_modes_[motor_idx] = mode;
            // 模式切换时，重置控制字状态
            if (!enable_requested_[motor_idx]) {
                control_word_states_[motor_idx].store(CONTROL_WORD_SWITCH_ON, std::memory_order_release);
                pending_control_words_[motor_idx] = CONTROL_WORD_SWITCH_ON;
            }
        }
        
        std::cout << "✓ CSV模式速度参数初始化完成（所有速度=0），模式已设置" << std::endl;
    } else {
        // ⭐ 其他模式：直接设置模式（已移除 data_mutex_）
        if (motor_id == 0xFF) {
            for (size_t i = 0; i < motor_count_; ++i) {
                const auto& cfg = motor_configs_[i];
                if (cfg.vendor_id == kGatewayVendorId && cfg.product_code == kGatewayProductCode) {
                    continue;
                }
                disarmLeavingMode(i, current_modes_[i], mode);
                current_modes_[i] = mode;
                if (!enable_requested_[i]) {
                    control_word_states_[i] = CONTROL_WORD_SWITCH_ON;
                    pending_control_words_[i] = CONTROL_WORD_SWITCH_ON;
                }
            }
            std::cout << "Set operation_mode=" << operationModeToString(mode) 
                      << " for all motors" << std::endl;
        } else if (motor_id < motor_count_) {
            disarmLeavingMode(motor_id, current_modes_[motor_id], mode);
            current_modes_[motor_id] = mode;
            if (!enable_requested_[motor_id]) {
                control_word_states_[motor_id] = CONTROL_WORD_SWITCH_ON;
                pending_control_words_[motor_id] = CONTROL_WORD_SWITCH_ON;
            }
            std::cout << "Set operation_mode=" << operationModeToString(mode) 
                      << " for motor " << (int)motor_id << std::endl;
        }
    }
    
    // ⭐ 模式切换后，等待模式生效并重新使能
    if (activated_) {
        std::cout << "Waiting for mode switch to take effect..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        std::cout << "Enabling all motors after mode change..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        try {
            setEnable(0xFF, true);
        } catch (const std::exception& e) {
            std::cerr << "Warning: Exception when enabling motors: " << e.what() << std::endl;
        }
    }

    if (runtime_logger_) {
        RuntimeLogEvent ev{};
        ev.timestamp_ns = getMonotonicTimeNs();
        ev.motor_id = motor_id;
        ev.type = RuntimeLogEventType::SERVO_COMMAND;
        ev.operation_mode = static_cast<int8_t>(mode);
        std::snprintf(ev.detail, sizeof(ev.detail), "set_operation_mode mode=%s",
                      operationModeToString(mode).c_str());
        const size_t log_motor = (motor_id == 0xFF || motor_id >= motor_count_) ? 0 : motor_id;
        fillRuntimeEventTargets(ev, log_motor);
        runtime_logger_->queueCommand(ev);
    }
    
    return true;
}

OperationMode EtherCATServo::getOperationMode(uint8_t motor_id) const
{
    if (motor_id >= motor_count_ || isGateway(motor_configs_[motor_id])) {
        return OperationMode::NONE;
    }
    return current_modes_[motor_id];
}

void EtherCATServo::setTargetPosition(uint8_t motor_id, int32_t position, bool override_filter)
{
    if (motor_id < motor_count_) {
        // ⭐ 使用 pending_commands_ 缓冲区，避免阻塞实时循环
        auto& pending = pending_commands_[motor_id];
        pending.position.store(position, std::memory_order_relaxed);
        pending.position_valid.store(true, std::memory_order_release);
        
        if (override_filter) {
            position_override_active_[motor_id] = true;
        }
        
        // 检测异常大值（仅在第一次检测到异常时输出）
        static std::map<uint8_t, bool> error_logged;
        if (std::abs(position) > 100000000 && !error_logged[motor_id]) {
            double position_deg = pulseToDegree(position, motor_id);
            std::cerr << "⚠️ 警告：电机" << (int)motor_id << " 目标位置异常大: " 
                      << position << " 脉冲 (" << std::fixed << std::setprecision(2) 
                      << position_deg << "°)" << std::endl;
            error_logged[motor_id] = true;
        }
    }
}

void EtherCATServo::setTargetVelocity(
    uint8_t motor_id, int32_t velocity, bool override_idle, SetpointSource source)
{
    if (motor_id < motor_count_) {
        auto& pending = pending_commands_[motor_id];
        pending.velocity.store(velocity, std::memory_order_relaxed);
        pending.velocity_valid.store(true, std::memory_order_release);

        if (override_idle) {
            velocity_override_active_[motor_id] = true;
        }

        if (source == SetpointSource::JobInternal) {
            markJobInternalCommand(velocity_cmd_freshness_[motor_id]);
        } else if (csv_cmd_watchdog_ &&
                   current_modes_[motor_id] == OperationMode::CYCLIC_SYNC_VELOCITY)
        {
            armExternalCommand(
                velocity_cmd_freshness_[motor_id],
                getMonotonicTimeNs(),
                external_cmd_watchdog_ns_);
        }
    }
}

void EtherCATServo::setIdleInputVelocity(uint8_t motor_id, int32_t velocity)
{
    if (motor_id >= motor_count_) {
        std::cerr << "Invalid motor ID: " << (int)motor_id << std::endl;
        return;
    }
    
    // ⭐ 已移除 data_mutex_，使用原子操作
    // ⭐ 更新静止速度（通常为0）
    idle_input_velocities_[motor_id] = velocity;
    
    // 同时更新目标速度（使用 pending_commands_）
    auto& pending = pending_commands_[motor_id];
    pending.velocity.store(velocity, std::memory_order_relaxed);
    pending.velocity_valid.store(true, std::memory_order_release);
    
    // ⭐ 清除覆盖标志，恢复正常的静止状态处理
    velocity_override_active_[motor_id] = false;
    
    // ⭐ 减少日志输出，避免阻塞
    // double velocity_deg = pulsePerSecToDegreePerSec(velocity, motor_id);
    // std::cout << "Motor " << (int)motor_id 
    //           << " idle input velocity set to: " << velocity << " 脉冲/s = " 
    //           << std::fixed << std::setprecision(3) << velocity_deg << "°/s" << std::endl;
}

void EtherCATServo::setTargetTorque(uint8_t motor_id, int16_t torque, SetpointSource source)
{
    if (motor_id < motor_count_) {
        auto& pending = pending_commands_[motor_id];
        pending.torque.store(torque, std::memory_order_relaxed);
        pending.torque_valid.store(true, std::memory_order_release);

        if (source == SetpointSource::JobInternal) {
            markJobInternalCommand(torque_cmd_freshness_[motor_id]);
        } else {
            armExternalCommand(
                torque_cmd_freshness_[motor_id],
                getMonotonicTimeNs(),
                external_cmd_watchdog_ns_);
        }
    }
}

int32_t EtherCATServo::getPosition(uint8_t motor_id) const
{
    return seqlockReadIndex(pdo_cache_seq_, last_actual_positions_, motor_id, int32_t{0});
}

int32_t EtherCATServo::getTargetPosition(uint8_t motor_id) const
{
    if (motor_id < motor_count_) {
        return target_positions_[motor_id].load(std::memory_order_acquire);
    }
    return 0;
}

int32_t EtherCATServo::getVelocity(uint8_t motor_id) const
{
    return seqlockReadIndex(pdo_cache_seq_, last_actual_velocities_, motor_id, int32_t{0});
}

int32_t EtherCATServo::getTargetVelocity(uint8_t motor_id) const
{
    if (motor_id < motor_count_) {
        return target_velocities_[motor_id].load(std::memory_order_acquire);
    }
    return 0;
}

int16_t EtherCATServo::getTorque(uint8_t motor_id) const
{
    return seqlockReadIndex(pdo_cache_seq_, last_actual_torques_, motor_id, int16_t{0});
}

int32_t EtherCATServo::getSensorForce2020(uint8_t motor_id) const
{
    return seqlockReadIndex(pdo_cache_seq_, last_sensor_force_2020_, motor_id, int32_t{0});
}

int32_t EtherCATServo::getMotorEncoder2021(uint8_t motor_id) const
{
    return seqlockReadIndex(pdo_cache_seq_, last_motor_encoder_2021_, motor_id, int32_t{0});
}

int16_t EtherCATServo::getTargetTorque(uint8_t motor_id) const
{
    if (motor_id < motor_count_) {
        // ⭐ 从 pending_commands_ 缓冲区读取最近一次设置的目标力矩
        return pending_commands_[motor_id].torque.load(std::memory_order_acquire);
    }
    return 0;
}

std::vector<MotorStateData> EtherCATServo::getMotorStates() const
{
    std::vector<MotorStateData> states;
    states.reserve(motor_count_);  // 预分配内存以提高性能
    
    // 安全检查：确保基本条件满足
    if (!activated_ || motor_count_ == 0) {
        return states;
    }

    // ⭐ 只处理前7个电机（不处理网关）
    size_t state_count = std::min<size_t>(7, motor_count_);

    std::vector<uint16_t> status_words(state_count);
    std::vector<int8_t> mode_displays(state_count);
    std::vector<int32_t> positions(state_count);
    std::vector<int32_t> velocities(state_count);
    std::vector<int16_t> torques(state_count);
    std::vector<int32_t> sensor_forces(state_count);
    std::vector<int32_t> motor_encoders(state_count);
    std::vector<uint16_t> error_codes(state_count);
    std::vector<bool> enables(state_count);

    for (;;) {
        const uint64_t s = pdo_cache_seq_.readBegin();
        for (size_t i = 0; i < state_count; ++i) {
            if (i >= pdo_offsets_.size()) {
                break;
            }
            status_words[i] = last_status_words_[i];
            mode_displays[i] = last_operation_mode_displays_[i];
            positions[i] = last_actual_positions_[i];
            velocities[i] = last_actual_velocities_[i];
            torques[i] = last_actual_torques_[i];
            sensor_forces[i] = last_sensor_force_2020_[i];
            motor_encoders[i] = last_motor_encoder_2021_[i];
            error_codes[i] = last_error_codes_[i];
            enables[i] = static_cast<bool>(enable_requested_[i]);
        }
        if (!pdo_cache_seq_.readRetry(s)) {
            break;
        }
    }

    for (size_t i = 0; i < state_count; ++i) {
        // 边界检查
        if (i >= pdo_offsets_.size()) {
            break;
        }
        
        MotorStateData state;
        state.motor_id = i;
        state.operation_mode = current_modes_[i];
        state.enabled = enables[i] && isCiA402OperationEnabled(status_words[i]);

        state.status_word = status_words[i];
        state.fault = isCiA402Fault(state.status_word);
        
        state.actual_position = positions[i];
        state.target_position = target_positions_[i].load(std::memory_order_acquire);
        
        state.actual_velocity = velocities[i];
        state.target_velocity = target_velocities_[i].load(std::memory_order_acquire);
        
        state.actual_torque = torques[i];
        state.sensor_force_2020 = sensor_forces[i];
        state.motor_encoder_2021 = motor_encoders[i];
        state.error_code = error_codes[i];
        state.target_torque = target_torques_[i].load(std::memory_order_acquire);
        
        state.operation_mode_display = mode_displays[i];
        
        // 读取数字 I/O （这个不常用，可以保留直接读取或者也做缓存）
        if (domain_in_pd_ && pdo_offsets_[i].digital_inputs != 0) {
            const auto& offsets = pdo_offsets_[i];
            state.digital_inputs = EC_READ_U32(domain_in_pd_ + offsets.digital_inputs);
        } else {
            state.digital_inputs = 0;
        }
        state.digital_outputs = 0; // 输出由主站控制
        
        states.push_back(state);
    }
    
    return states;
}

std::string EtherCATServo::operationModeToString(OperationMode mode) const
{
    switch (mode) {
        case OperationMode::CYCLIC_SYNC_POSITION: return "CSP";
        case OperationMode::CYCLIC_SYNC_VELOCITY: return "CSV";
        case OperationMode::CYCLIC_SYNC_TORQUE: return "CST";
        // case OperationMode::PROFILE_POSITION: return "PP";
        // case OperationMode::PROFILE_VELOCITY: return "PV";
        // case OperationMode::PROFILE_TORQUE: return "PT";
        // case OperationMode::HOMING: return "HM";
        // case OperationMode::INTERPOLATED_POSITION: return "IP";
        default: return "NONE";
    }
}

OperationMode EtherCATServo::stringToOperationMode(const std::string& mode_str) const
{
    if (mode_str == "CSP") return OperationMode::CYCLIC_SYNC_POSITION;
    if (mode_str == "CSV") return OperationMode::CYCLIC_SYNC_VELOCITY;
    if (mode_str == "CST") return OperationMode::CYCLIC_SYNC_TORQUE;
    // if (mode_str == "PP") return OperationMode::PROFILE_POSITION;
    // if (mode_str == "PV") return OperationMode::PROFILE_VELOCITY;
    // if (mode_str == "PT") return OperationMode::PROFILE_TORQUE;
    // if (mode_str == "HM") return OperationMode::HOMING;
    // if (mode_str == "IP") return OperationMode::INTERPOLATED_POSITION;
    return OperationMode::NONE;
}

/****************************************************************************/
// 分布式时钟（DC）功能
/****************************************************************************/

void EtherCATServo::setIdleInputPosition(uint8_t motor_id, int32_t position)
{
    if (motor_id >= motor_count_) {
        std::cerr << "Invalid motor ID: " << (int)motor_id << std::endl;
        return;
    }
    
    // ⭐ 已移除 data_mutex_，使用原子操作
    // ⭐ 更新静止输入位置（直接更新，无需buffer）
    idle_input_positions_[motor_id] = position;
    
    // 同时更新目标位置，确保无缝切换（使用 pending_commands_）
    auto& pending = pending_commands_[motor_id];
    pending.position.store(position, std::memory_order_relaxed);
    pending.position_valid.store(true, std::memory_order_release);
    
    // ⭐ 清除覆盖标志，恢复正常的静止状态处理
    // 轨迹完成后，电机应该使用 idle_input_positions_ 保持位置
    position_override_active_[motor_id] = false;
    
    // ⭐ 减少日志输出，避免阻塞
    // double position_deg = pulseToDegree(position, motor_id);
    // std::cout << "Motor " << (int)motor_id 
    //           << " idle input position set to: " << position << " 脉冲 = " 
    //           << std::fixed << std::setprecision(4) << position_deg << "°" << std::endl;
}

/****************************************************************************/
// DC时间基准对齐函数
/****************************************************************************/

/**
 * @brief 对齐DC时间基准到系统时间
 * @param system_time_ns 系统时间（纳秒，64位）
 * 
 * @details
 * 此函数用于在DC参考时钟不可用时，将DC时间的64位累积变量对齐到系统时间。
 * 这样当DC恢复时，其64位时间将从系统时间开始累积，而不是从0开始。
 * 
 * 工作原理：
 * 1. ecrt_master_reference_clock_time() 只返回32位时间（低32位）
 * 2. 需要通过检测溢出来手动扩展为64位时间
 * 3. dc_high_bits_ 存储高32位，last_time_low_ 存储上次的低32位
 * 4. 当DC不可用时，使用系统时间初始化这两个变量
 * 5. 当DC恢复时，64位时间将从系统时间继续累积，保持时间连续性
 * 
 * @note
 * - 此函数应在DC不可用且需要使用系统时间作为后备时调用
 * - 调用后，下次成功获取DC时间时，64位时间将从对齐的系统时间开始
 * - dc_high_bits_ 和 last_time_low_ 是文件作用域静态变量，与 getReferenceClockTime() 共享
 */


// DC 时间 32→64 位扩展状态（getReferenceClockTime / alignDcTimeBase 共用）
namespace {
uint32_t dc_high_bits_ = 0;
uint32_t last_time_low_ = 0;
bool dc_high_bits_aligned_ = false;
}  // namespace

void EtherCATServo::setApplicationTime(uint64_t app_time_ns)
{
    if (!activated_) return;
    
    // 将应用时间写入主站
    // 这对于分布式时钟同步至关重要
    ecrt_master_application_time(master_, app_time_ns);
}

void EtherCATServo::syncReferenceClock(uint64_t time_ns)
{
    if (!activated_) return;
    
    // 使用主站内部参考时钟同步（更稳健）
    (void)time_ns; // 参数保留但不使用
    ecrt_master_sync_reference_clock(master_);
}

void EtherCATServo::syncSlaveClocks()
{
    if (!activated_) return;
    
    // 同步所有从站的时钟
    ecrt_master_sync_slave_clocks(master_);
}

uint64_t EtherCATServo::getReferenceClockTime() const
{
    if (!activated_) return 0;
    
    // ==========================================================
    // ⭐ 64位时间累积变量（文件作用域，与 alignDcTimeBase() 共享）
    // 用于将 32 位 DC 时间扩展为连续的 64 位时间戳
    // ==========================================================
    // 注意：dc_high_bits_ 和 last_time_low_ 已在文件作用域定义
    
    // DC 可用性跟踪变量（保持不变）
    static bool dc_available = false;
    static int retry_counter = 0;
    static int consecutive_failures = 0;
    static int consecutive_successes = 0;
    const int RETRY_INTERVAL = 5000;
    const int MAX_CONSECUTIVE_FAILURES = 10;
    const int INITIAL_RETRY_INTERVAL = 100;
    const int REQUIRED_SUCCESSES = 10;
    
    // 如果之前检测到DC不可用，定期重试
    if (!dc_available) {
        retry_counter++;
        int actual_retry_interval = (consecutive_failures == 0) ? 
                                    INITIAL_RETRY_INTERVAL :
                                    ((consecutive_failures >= MAX_CONSECUTIVE_FAILURES) ? 
                                     RETRY_INTERVAL * 10 : RETRY_INTERVAL);
        
        if (retry_counter < actual_retry_interval) {
            return 0;
        }
        retry_counter = 0;
    }
    
    
    // 保存并清除errno
    int saved_errno = errno;
    errno = 0;
    
    // 获取DC参考时钟时间
    uint32_t time_low = 0;
    int ret = ecrt_master_reference_clock_time(master_, &time_low);
    

    if (ret != 0) {
        // DC未同步或参考时钟未准备好
        dc_available = false;
        consecutive_failures++;
        consecutive_successes = 0;
        errno = 0;
        
        // dc_high_bits_aligned_ = false;
        
        return 0;
    }

    // 如果 DC 刚刚恢复，且尚未对齐，则执行基准校准
    if (!dc_high_bits_aligned_) {
        // 获取系统时间用于对齐
        struct timespec ts;
        // 务必确保这里使用的时钟源与 cyclicTask 一致 (例如 CLOCK_MONOTONIC)
        clock_gettime(CLOCK_MONOTONIC, &ts); 
        uint64_t sys_now = (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;

        // 将 DC 的高位设置为系统时间的高位，强行拉平时间基准
        dc_high_bits_ = (uint32_t)(sys_now >> 32);
        
        last_time_low_ = time_low;
        dc_high_bits_aligned_ = true;
    }

    
    // --- DC同步成功，执行 64 位时间扩展 ---
    
    // 1. 检测低 32 位是否溢出 (从 0xFFFFFFFF 翻转到 0x00000000)
    // 只有当 time_low < last_time_low_ 且 last_time_low_ 不为 0 时才认为溢出。
    // （在启动时 last_time_low_ 为 0，避免误判第一次获取的时间）
    if (time_low < last_time_low_ && last_time_low_ != 0) {
        dc_high_bits_++; // 增加高 32 位
    }
    
    // 2. 更新上次低位时间
    last_time_low_ = time_low;
    
    // 3. 构建完整的 64 位时间
    uint64_t full_time_ns = (static_cast<uint64_t>(dc_high_bits_) << 32) | time_low;
    
    // --- 稳定判断逻辑（保持不变）---
    consecutive_successes++;
    consecutive_failures = 0;
    
    if (consecutive_successes >= REQUIRED_SUCCESSES) {
        dc_available = true;
    } else {
        dc_available = false;
        retry_counter = 0;
    }
    
    errno = saved_errno;
    
    // 返回完整的 64 位时间戳
    return full_time_ns;
}

/****************************************************************************/
// 状态检查功能
/****************************************************************************/

void EtherCATServo::checkDomainState()
{
    if (!activated_ || !domain_out_ || !domain_in_) return;
    
    ec_domain_state_t ds_out{};
    ec_domain_state_t ds_in{};
    ecrt_domain_state(domain_out_, &ds_out);
    ecrt_domain_state(domain_in_, &ds_in);

    if (ds_out.working_counter != domain_out_state_.working_counter ||
        ds_in.working_counter != domain_in_state_.working_counter) {
        std::cout << "Domain: WC_out " << ds_out.working_counter
                  << " WC_in " << ds_in.working_counter << std::endl;
    }
    if (ds_out.wc_state != domain_out_state_.wc_state ||
        ds_in.wc_state != domain_in_state_.wc_state) {
        std::cout << "Domain: State_out " << static_cast<int>(ds_out.wc_state)
                  << " State_in " << static_cast<int>(ds_in.wc_state) << std::endl;
    }

    domain_out_state_ = ds_out;
    domain_in_state_ = ds_in;
    domain_state_ = ds_out;
}

void EtherCATServo::checkMasterState()
{
    if (!activated_) return;
    
    ec_master_state_t ms;
    ecrt_master_state(master_, &ms);

    if (ms.slaves_responding != master_state_.slaves_responding) {
        std::cout << ms.slaves_responding << " slave(s) responding." << std::endl;
    }
    if (ms.al_states != master_state_.al_states) {
        std::cout << "AL states: 0x" << std::hex << (int)ms.al_states 
                  << std::dec << std::endl;
    }
    if (ms.link_up != master_state_.link_up) {
        std::cout << "Link is " << (ms.link_up ? "up" : "down") << std::endl;
    }

    master_state_ = ms;
}

bool EtherCATServo::hasActiveMotors() const
{
    // 只要EtherCAT主站激活就使用高频率
    return activated_;
}

bool EtherCATServo::checkSlaveStates() const
{
    if (!activated_ || !master_) return true;
    
    bool all_ok = true;
    ec_slave_config_state_t sc_state;
    
    std::cout << "检查从站状态:" << std::endl;
    // ⭐ 只检查前7个电机（不检查网关）
    size_t check_count = std::min<size_t>(7, motor_count_);
    for (size_t i = 0; i < check_count; ++i) {
        if (slave_configs_[i]) {
            ecrt_slave_config_state(slave_configs_[i], &sc_state);
            
            // 检查 AL 状态
            uint8_t al_state = sc_state.al_state;
            const char* state_str = "UNKNOWN";
            if (al_state == EC_AL_STATE_INIT) state_str = "INIT";
            else if (al_state == EC_AL_STATE_PREOP) state_str = "PREOP";
            else if (al_state == EC_AL_STATE_SAFEOP) state_str = "SAFEOP";
            else if (al_state == EC_AL_STATE_OP) state_str = "OP";
            
            bool is_ok = (al_state == EC_AL_STATE_OP);
            if (!is_ok) all_ok = false;
            
            std::cout << "  电机 " << i << ": AL=" << state_str 
                      << " (0x" << std::hex << (int)al_state << std::dec << ")" << std::endl;
            
            // 如果不在 OP 状态，打印详细信息
            if (!is_ok) {
                std::cout << "    ⚠ 电机 " << i << " 处于异常状态 (AL状态: " << state_str << ")" << std::endl;
            }
        }
    }
    
    if (all_ok) {
        std::cout << "✓ 所有电机(0-6)处于 OP 状态" << std::endl;
    } else {
        std::cout << "⚠ 部分电机处于异常状态，请检查!" << std::endl;
    }
    
    return all_ok;
}

void EtherCATServo::diagnoseSlaveAlStates() const
{
    if (!activated_ || !master_) {
        return;
    }
    for (size_t i = 0; i < motor_count_; ++i) {
        if (!slave_configs_[i]) {
            continue;
        }
        ec_slave_config_state_t sc{};
        ecrt_slave_config_state(slave_configs_[i], &sc);
        std::cout << "[IgH] slave " << i
                  << " al=0x" << std::hex << static_cast<int>(sc.al_state) << std::dec
                  << " online=" << (sc.online ? 1 : 0)
                  << " op=" << (sc.operational ? 1 : 0) << std::endl;
    }
}

bool EtherCATServo::areAllSlavesInOP() const
{
    if (!activated_ || !master_ || !domain_out_ || !domain_in_) return false;

    ec_domain_state_t ds_out{};
    ec_domain_state_t ds_in{};
    ecrt_domain_state(domain_out_, &ds_out);
    ecrt_domain_state(domain_in_, &ds_in);
    if (ds_out.working_counter == 0 || ds_in.working_counter == 0 ||
        ds_out.wc_state != EC_WC_COMPLETE || ds_in.wc_state != EC_WC_COMPLETE) {
        return false;
    }

    // 使用 online + operational（与周期诊断一致），al_state 在运行中可能短暂抖动为 0x2
    for (size_t i = 0; i < motor_count_; ++i) {
        if (!slave_configs_[i]) {
            return false;
        }
        ec_slave_config_state_t sc_state;
        ecrt_slave_config_state(slave_configs_[i], &sc_state);
        if (!sc_state.online || !sc_state.operational) {
            return false;
        }
    }
    return true;
}

bool EtherCATServo::setPositionFilterSize(size_t size)
{
    // 验证输入范围
    if (size < 1 || size > 11) {
        std::cerr << "Invalid filter size: " << size 
                  << " (must be 1-11)" << std::endl;
        return false;
    }
    

    position_filter_size_ = size;
    
    // 保留此函数以兼容旧代码，但实际不再使用filter
    
    std::cout << "⚠ Position filter已废弃（停止状态直接使用idle位置，无滤波延迟）" << std::endl;
    std::cout << "  position_filter_size保留为: " << size << " (兼容性)" << std::endl;
    
    return true;
}

bool EtherCATServo::initializePositionsFromSDO()
{
    if (!initialized_) {
        std::cerr << "SDO初始化必须在initialize()之后调用" << std::endl;
        return false;
    }
    if (activated_) {
        // After ecrt_master_activate(), mailbox SDO without cyclic Job can block forever.
        std::cerr << "initializePositionsFromSDO: master already activated; "
                  << "call before activate() (IgH mailbox SDO)" << std::endl;
        return false;
    }

    std::cout << "[IgH] SDO read initial positions (0x6064)" << std::endl;
    
    const int READ_COUNT = 5;  // 每个电机读取5次
    int success_count = 0;
    
    for (size_t i = 0; i < motor_count_; ++i) {
        const auto& cfg = motor_configs_[i];
        
        // 跳过网关
        if (cfg.vendor_id == kGatewayVendorId && cfg.product_code == kGatewayProductCode) {
            continue;
        }
        
        std::vector<int32_t> read_values;
        
        // 读取多次，使用cfg.position而不是循环索引i
        for (int attempt = 0; attempt < READ_COUNT; ++attempt) {
            int32_t actual_pos = 0;
            size_t result_size = sizeof(actual_pos);
            uint32_t abort_code = 0;
            
            int ret = ecrt_master_sdo_upload(master_, cfg.position, 0x6064, 0,
                                            reinterpret_cast<uint8_t*>(&actual_pos),
                                            sizeof(actual_pos), &result_size, &abort_code);
            
            if (ret == 0 && result_size == sizeof(actual_pos)) {
                read_values.push_back(actual_pos);
            } else if (attempt + 1 == READ_COUNT) {
                std::cerr << "[IgH] axis " << i << " 0x6064 upload failed ret=" << ret
                          << " abort=0x" << std::hex << abort_code << std::dec << std::endl;
            }
            
            // 每次读取之间短暂延迟
            if (attempt < READ_COUNT - 1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        
        // 选择最合理的值
        if (!read_values.empty()) {
            int32_t final_pos = 0;
            
            if (read_values.size() >= 3) {
                // 有至少3个值，使用中位数（更稳健）
                std::vector<int32_t> sorted_values = read_values;
                std::sort(sorted_values.begin(), sorted_values.end());
                final_pos = sorted_values[sorted_values.size() / 2];
            } else {
                // 少于3个值，使用第一个非0值或最后一个值
                bool found_nonzero = false;
                for (int32_t val : read_values) {
                    if (val != 0) {
                        final_pos = val;
                        found_nonzero = true;
                        break;
                    }
                }
                if (!found_nonzero) {
                    final_pos = read_values.back();
                }
            }
            
            // 设置初始位置
            idle_input_positions_[i] = final_pos;
            target_positions_[i].store(final_pos, std::memory_order_release);
            
            // 同步更新 PDO 缓存（seqlock），确保 CSP 模式切换时能正确获取位置
            {
                SeqLockWriter guard(pdo_cache_seq_);
                last_actual_positions_[i] = final_pos;
            }
            
            success_count++;
        } else {
            std::cerr << "[IgH] axis " << i << " initial position read failed; keep 0"
                      << std::endl;
        }
    }
    
    std::cout << "[IgH] initial positions ready (" << success_count << "/"
              << motor_count_ << ")" << std::endl;
    
    return success_count > 0;
}

namespace {

bool uploadSdoUint32(ec_master_t* master, uint16_t slave_pos, uint16_t index, uint8_t subindex,
                       uint32_t& value)
{
    value = 0;
    size_t result_size = sizeof(value);
    uint32_t abort_code = 0;
    const int ret = ecrt_master_sdo_upload(
        master, slave_pos, index, subindex,
        reinterpret_cast<uint8_t*>(&value), sizeof(value), &result_size, &abort_code);
    return ret == 0 && result_size == sizeof(value);
}

bool uploadSdoU8(ec_master_t* master, uint16_t slave_pos, uint16_t index, uint8_t subindex,
                   uint8_t& value)
{
    value = 0;
    size_t result_size = sizeof(value);
    uint32_t abort_code = 0;
    const int ret = ecrt_master_sdo_upload(
        master, slave_pos, index, subindex,
        reinterpret_cast<uint8_t*>(&value), sizeof(value), &result_size, &abort_code);
    return ret == 0 && result_size == sizeof(value);
}

bool downloadSdoU8(ec_master_t* master, uint16_t slave_pos, uint16_t index, uint8_t subindex,
                   uint8_t value)
{
    uint32_t abort_code = 0;
    const int ret = ecrt_master_sdo_download(
        master, slave_pos, index, subindex,
        reinterpret_cast<const uint8_t*>(&value), sizeof(value), &abort_code);
    return ret == 0;
}

}  // namespace

bool EtherCATServo::tryLoadKinematicsFromSdo()
{
    if (!initialized_ || !master_ || motor_count_ == 0) {
        return false;
    }

    std::vector<MotorKinematicsParams> params(motor_count_);
    bool updated = false;
    bool logged_profile_kin = false;

    for (size_t i = 0; i < motor_count_; ++i) {
        params[i] = MotorKinematics::get(i);
        const auto& cfg = motor_configs_[i];

        if (cfg.vendor_id == kGatewayVendorId && cfg.product_code == kGatewayProductCode) {
            continue;
        }

        const bool is_sjd17 = (cfg.vendor_id == kSjd17VendorId &&
                                   cfg.product_code == kSjd17ProductCode);
        const bool is_jmdt = isCoolDriveJmdtMotor(cfg);

        if (is_sjd17 || is_jmdt) {
            const MotorProfile* profile = MotorProfileRegistry::findByIdentity(
                cfg.vendor_id, cfg.product_code);
            if (profile) {
                params[i].gear_ratio = profile->kinematics.gear_ratio;
                params[i].torque_gear_ratio = profile->kinematics.torque_gear_ratio;
                params[i].encoder_resolution = profile->kinematics.encoder_resolution;
                params[i].output_side_encoder = profile->kinematics.output_side_encoder;
                updated = true;
                if (!logged_profile_kin) {
                    logged_profile_kin = true;
                    std::cout << "[IgH] kinematics from profile ("
                              << (is_jmdt ? "JMDT" : "SJD17")
                              << " enc=" << static_cast<int>(profile->kinematics.encoder_resolution)
                              << " ratio=" << profile->kinematics.gear_ratio << ")"
                              << std::endl;
                }
            }
            continue;
        }

        uint32_t gear_raw = 0;
        uint32_t pos_mode_raw = 0;
        uint32_t max_current_raw = 0;
        const bool got_gear = uploadSdoUint32(master_, cfg.position, 0x200e, 0x02, gear_raw);
        const bool got_pos_mode = uploadSdoUint32(master_, cfg.position, 0x2016, 0x00, pos_mode_raw);
        const bool got_max_current = uploadSdoUint32(master_, cfg.position, 0x6075, 0x00, max_current_raw);

        if (got_gear && gear_raw > 0) {
            params[i].gear_ratio = static_cast<double>(gear_raw);
            updated = true;
        }
        if (got_pos_mode) {
            // 新奇手册 0x2016：0=外圈262144，2=内圈65536×减速比
            if (pos_mode_raw == 0) {
                params[i].encoder_resolution = 262144.0;
                params[i].output_side_encoder = true;
                updated = true;
                std::cout << "  0x2016=0 → 外圈位置 enc=262144"
                          << "（速度仍用电机侧 gear×motor_enc，若 velocity_on_motor_encoder）"
                          << std::endl;
            } else if (pos_mode_raw == 2) {
                params[i].encoder_resolution = 65536.0;
                params[i].output_side_encoder = false;
                updated = true;
                std::cout << "  0x2016=2 → 内圈 enc=65536×减速比" << std::endl;
            } else {
                std::cout << "  ⚠ 0x2016=" << pos_mode_raw
                          << " 未知位置模式，沿用 yaml" << std::endl;
            }
        }
        if (got_max_current && max_current_raw > 0) {
            params[i].max_current_ma = static_cast<double>(max_current_raw);
            updated = true;
        }

        std::cout << "[电机" << i << "] SDO 读值:"
                  << " 0x2016=" << (got_pos_mode ? std::to_string(pos_mode_raw) : "N/A")
                  << " 0x200E:2=" << (got_gear ? std::to_string(gear_raw) : "N/A")
                  << " 0x6075=" << (got_max_current ? std::to_string(max_current_raw) : "N/A")
                  << " mA"
                  << std::endl;
    }

    if (updated) {
        MotorKinematics::setParams(params);
        if (motor_count_ > 0 && !isGateway(motor_configs_[0])) {
            std::cout << "[IgH] kinematics applied[0]: " << MotorKinematics::describe(0)
                      << std::endl;
        }
    } else {
        std::cout << "[IgH] kinematics SDO unavailable; keep yaml/profile" << std::endl;
    }
    return updated;
}

void EtherCATServo::alignDcTimeBase(uint64_t system_time_ns)
{
    // 将64位系统时间分解为高32位和低32位
    dc_high_bits_ = static_cast<uint32_t>(system_time_ns >> 32);      // 提取高32位
    last_time_low_ = static_cast<uint32_t>(system_time_ns & 0xFFFFFFFF);  // 提取低32位
    dc_high_bits_aligned_ = true;
    // 这样当DC恢复时，getReferenceClockTime() 会从对齐的系统时间继续累积
    // 而不是从0开始，保持时间连续性
}

uint16_t EtherCATServo::readErrorCode(uint8_t motor_id) const
{
    if (!master_ || motor_id >= motor_count_) {
        return 0;
    }

    const auto& cfg = motor_configs_[motor_id];
    if (cfg.vendor_id == kGatewayVendorId && cfg.product_code == kGatewayProductCode) {
        return 0;
    }

    uint16_t error_code = 0;
    size_t result_size = sizeof(error_code);
    uint32_t abort_code = 0;
    const int ret = ecrt_master_sdo_upload(
        master_, cfg.position, 0x603F, 0,
        reinterpret_cast<uint8_t*>(&error_code),
        sizeof(error_code), &result_size, &abort_code);

    if (ret == 0 && result_size == sizeof(error_code)) {
        return error_code;
    }
    return 0;
}

void EtherCATServo::deactivate()
{
    IghMasterRuntime::instance().stop();
    if (master_ && activated_) {
        ecrt_master_deactivate(master_);
        domain_out_pd_ = nullptr;
        domain_in_pd_ = nullptr;
    }
    activated_ = false;
    safe_output_active_ = false;
}

bool EtherCATServo::isJobThreadRunning() const
{
    return IghMasterRuntime::instance().isJobThreadRunning();
}

uint32_t EtherCATServo::busCycleUs() const
{
    return IghMasterRuntime::instance().busCycleUs();
}

bool EtherCATServo::commFault() const
{
    return IghMasterRuntime::instance().commFault();
}

bool EtherCATServo::safeOutputRequired() const
{
    return IghMasterRuntime::instance().safeOutputRequired();
}

bool EtherCATServo::motionReenableAllowed() const
{
    return IghMasterRuntime::instance().motionReenableAllowed();
}

void EtherCATServo::clearCommFault()
{
    IghMasterRuntime::instance().clearCommFault();
    safe_output_active_ = false;
    explicit_fault_reset_cycles_.store(0U, std::memory_order_release);
    explicit_fault_reset_axis_.store(0xFFFFU, std::memory_order_release);
    disarmAllCommandFreshness();
}

void EtherCATServo::requestSafeOutput()
{
    IghMasterRuntime::instance().requestSafeOutput();
}

bool EtherCATServo::releaseSafeOutput()
{
    return startup_evidence_passed_ &&
        IghMasterRuntime::instance().releaseSafeOutput();
}

IghJobCycleDiag EtherCATServo::jobCycleDiag() const
{
    return IghMasterRuntime::instance().jobCycleDiag();
}

void EtherCATServo::disarmAllCommandFreshness()
{
    for (auto & s : torque_cmd_freshness_) {
        disarmCommandFreshness(s);
    }
    for (auto & s : velocity_cmd_freshness_) {
        disarmCommandFreshness(s);
    }
}

int32_t EtherCATServo::lastDcDiffNs() const
{
    return sync_handler_ ? sync_handler_->lastDcDiffNs() : 0;
}

bool EtherCATServo::isDcStatusValid() const
{
    return sync_handler_ && sync_handler_->isDcPllActive();
}

struct timespec EtherCATServo::getDcSleepSpec(uint64_t wakeup_time_ns) const
{
    if (sync_handler_) {
        return sync_handler_->getSleepSpec(wakeup_time_ns);
    }
    struct timespec ts{};
    ts.tv_sec = static_cast<time_t>(wakeup_time_ns / 1000000000ULL);
    ts.tv_nsec = static_cast<long>(wakeup_time_ns % 1000000000ULL);
    return ts;
}

uint64_t EtherCATServo::getDcApplicationTime() const
{
    if (sync_handler_) {
        return sync_handler_->getApplicationTime();
    }
    return getMonotonicTimeNs();
}

void EtherCATServo::checkExternalCommandFreshness()
{
    if (!initialized_ || external_cmd_watchdog_ns_ == 0U) {
        return;
    }
    if (IghMasterRuntime::instance().safeOutputRequired()) {
        return;
    }

    const uint64_t now = getMonotonicTimeNs();
    bool stale = false;

    for (size_t i = 0; i < motor_count_; ++i) {
        if (isGateway(motor_configs_[i])) {
            continue;
        }
        if (current_modes_[i] == OperationMode::CYCLIC_SYNC_TORQUE &&
            isExternalCommandStale(torque_cmd_freshness_[i], now))
        {
            pending_commands_[i].torque.store(0, std::memory_order_relaxed);
            pending_commands_[i].torque_valid.store(true, std::memory_order_release);
            target_torques_[i].store(0, std::memory_order_relaxed);
            disarmCommandFreshness(torque_cmd_freshness_[i]);
            stale = true;
        }
        if (csv_cmd_watchdog_ &&
            current_modes_[i] == OperationMode::CYCLIC_SYNC_VELOCITY &&
            isExternalCommandStale(velocity_cmd_freshness_[i], now))
        {
            pending_commands_[i].velocity.store(0, std::memory_order_relaxed);
            pending_commands_[i].velocity_valid.store(true, std::memory_order_release);
            target_velocities_[i].store(0, std::memory_order_relaxed);
            disarmCommandFreshness(velocity_cmd_freshness_[i]);
            stale = true;
        }
    }

    if (stale) {
        IghMasterRuntime::instance().raiseCommFault();
    }
}

void EtherCATServo::applyCommandContentionFallback()
{
    if (!initialized_) {
        return;
    }
    for (size_t i = 0; i < motor_count_; ++i) {
        if (isGateway(motor_configs_[i])) {
            continue;
        }
        if (current_modes_[i] == OperationMode::CYCLIC_SYNC_TORQUE &&
            contentionFallbackActive(torque_cmd_freshness_[i]))
        {
            pending_commands_[i].torque.store(0, std::memory_order_relaxed);
            pending_commands_[i].torque_valid.store(true, std::memory_order_release);
            target_torques_[i].store(0, std::memory_order_relaxed);
        }
        if (csv_cmd_watchdog_ &&
            current_modes_[i] == OperationMode::CYCLIC_SYNC_VELOCITY &&
            contentionFallbackActive(velocity_cmd_freshness_[i]))
        {
            pending_commands_[i].velocity.store(0, std::memory_order_relaxed);
            pending_commands_[i].velocity_valid.store(true, std::memory_order_release);
            target_velocities_[i].store(0, std::memory_order_relaxed);
        }
    }
}

bool EtherCATServo::runJobCycle(bool force_safe_output, uint64_t app_time_ns)
{
    // Job RT path（对齐天机）：setAppTime → RX → DC sync → safe|TX。
    // force_safe_output comes from runtime latch; also re-reads safeOutputRequired().
    // Must not block (no ROS log / mutex / heap) on this path.
    if (!initialized_ || !master_ || !domain_out_ || !domain_in_) {
        return false;
    }

    checkExternalCommandFreshness();

    const uint64_t t = (app_time_ns != 0ULL) ? app_time_ns : getMonotonicTimeNs();
    setApplicationTime(t);
    receiveData();

    bool rx_ok = false;
    {
        ec_domain_state_t ds_out{};
        ec_domain_state_t ds_in{};
        ecrt_domain_state(domain_out_, &ds_out);
        ecrt_domain_state(domain_in_, &ds_in);
        domain_out_state_ = ds_out;
        domain_in_state_ = ds_in;
        domain_state_ = ds_out;
        rx_ok = (ds_out.working_counter > 0) && (ds_in.working_counter > 0) &&
                (ds_out.wc_state == EC_WC_COMPLETE) && (ds_in.wc_state == EC_WC_COMPLETE);
    }

    processSync(t);

    const bool safe =
      force_safe_output || IghMasterRuntime::instance().safeOutputRequired();
    if (safe) {
        applySafeProcessImageOutputs();
    } else {
        safe_output_active_ = false;
        sendData();
    }
    return rx_ok;
}

void EtherCATServo::applySafeProcessImageOutputs()
{
    if (!initialized_ || !domain_out_pd_ || !domain_in_pd_ || !domain_out_ || !domain_in_ || !master_) {
        return;
    }

    const bool entering = !safe_output_active_;
    safe_output_active_ = true;

    const uint16_t reset_cycles =
        explicit_fault_reset_cycles_.load(std::memory_order_acquire);
    const uint16_t reset_axis =
        explicit_fault_reset_axis_.load(std::memory_order_acquire);

    for (size_t i = 0; i < motor_count_; ++i) {
        const auto & cfg = motor_configs_[i];
        if (cfg.vendor_id == kGatewayVendorId && cfg.product_code == kGatewayProductCode) {
            continue;
        }
        const auto & offsets = pdo_offsets_[i];
        const int32_t actual = getPosition(static_cast<uint8_t>(i));
        if (i < safe_latched_positions_.size()) {
            safe_latched_positions_[i] =
              latchSafePosition(entering, safe_latched_positions_[i], actual);
        }
        const auto safe = makeSafeProcessImageOutputs(
          (i < safe_latched_positions_.size()) ? safe_latched_positions_[i] : actual);

        if (entering) {
            fault_reset_phase_[i] = 0;
            fault_reset_counter_[i] = 0;
            enable_fsm_step_[i] = 0;
            enable_fsm_wait_[i] = 0;
        }

        desired_enable_[i] = false;
        enable_fsm_active_[i] = false;
        enable_requested_[i] = false;

        const uint16_t sw =
          (i < last_status_words_.size()) ? last_status_words_[i] : 0U;
        const bool axis_selected =
          reset_axis == 0x00FFU || reset_axis == static_cast<uint16_t>(i);
        const uint16_t safe_control_word = selectSafeControlWord(
          reset_cycles > 0U, axis_selected, isCiA402Fault(sw),
          (i < fault_reset_cw_.size()) ? fault_reset_cw_[i]
                                       : CONTROL_WORD_FAULT_RESET);

        control_word_states_[i].store(safe_control_word, std::memory_order_relaxed);
        target_positions_[i].store(safe.target_position, std::memory_order_relaxed);
        target_velocities_[i].store(safe.target_velocity, std::memory_order_relaxed);
        target_torques_[i].store(safe.target_torque, std::memory_order_relaxed);
        position_override_active_[i] = false;
        velocity_override_active_[i] = false;
        if (i < idle_input_positions_.size()) {
            idle_input_positions_[i] = safe.target_position;
        }

        // Offset 0 is valid (first entry in a domain); only skip unset fields.
        if (offsets.control_word != kPdoOffsetUnset) {
            EC_WRITE_U16(domain_out_pd_ + offsets.control_word, safe_control_word);
        }
        if (offsets.target_position != kPdoOffsetUnset) {
            EC_WRITE_S32(domain_out_pd_ + offsets.target_position, safe.target_position);
        }
        if (offsets.target_velocity != 0) {
            EC_WRITE_S32(domain_out_pd_ + offsets.target_velocity, safe.target_velocity);
        }
        if (offsets.target_torque != 0) {
            EC_WRITE_S16(domain_out_pd_ + offsets.target_torque, safe.target_torque);
        }
        if (offsets.operation_mode != 0) {
            EC_WRITE_S8(
              domain_out_pd_ + offsets.operation_mode,
              static_cast<int8_t>(current_modes_[i]));
        }
    }

    if (reset_cycles > 0U) {
        const uint16_t remaining =
            explicit_fault_reset_cycles_.fetch_sub(
                1U, std::memory_order_acq_rel) - 1U;
        if (remaining == 0U) {
            explicit_fault_reset_axis_.store(
                0xFFFFU, std::memory_order_release);
        }
    }

    ecrt_domain_queue(domain_out_);
    ecrt_master_send(master_);
}

}  // namespace ethercat_joint
