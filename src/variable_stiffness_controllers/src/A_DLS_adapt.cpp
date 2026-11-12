


/**
 * A_DLS_adapt.cpp
 * 各向异性自适应任务分级控制器 — 论文 v5.12 Algorithm 1 完整实现
 *
 * 论文: "面向未知接触环境的7-DOF冗余机械臂各向异性自适应任务分级策略" v5.12
 *
 * 所有算法子模块内联，公式编号与论文一一对应。
 * 支持 M1–M4 四种对比方法，诊断 93 字段兼容现有分析脚本。
 *
 * 与论文 v5.12 的一致性:
 *   §2    GMO: Eq (2.9) 离散更新                                    ✓
 *   §3.1  FDI: Eq (3.2)–(3.2a) k* 反向遍历                          ✓
 *   §3.3  SVD投影: Eq (3.6–3.7) 死区 + Eq (3.7a) 自适应阈值          ✓
 *         → alpha_th 不含 m_aff（论文 Eq 3.7a 含m_aff，但无碰撞时
 *           m_aff=0 → α_th→ε → sigmoid≈0.5 → λ²≈(lb+lm)/2虚假阻尼
 *           实现修正：始终用全关节噪声上界保证无碰撞时λ²→lb_）        [impl fix]
 *   §4.2  阻尼优化: Eq (4.9) 凸组合 + (4.12-4.13) sigmoid           ✓
 *   §4.5  非对称 EMA: Eq (4.19)–(4.19a) 逐元素快攻慢退             ✓
 *   §4.5  策略B: V1 符号追踪                                        ✓
 *   §5    伪逆: Eq (4.4) J^T(JJ^T+Λ)^(-1)                          ✓
 *   §6.1.1 参考调制 v5.12: β由笛卡尔力F=(JJ^T)^{-1}Jr̃驱动，        ✓
 *          逐方向clip → ẽ_eff=(1-β)⊙ẽ (Eq 6.1a-1, 6.1a-2, 6.1b)
 *          (v5.11 SVD-β几何纠缠已修正)
 *   §6.2  零空间柔顺: Eq (6.2)–(6.2b)                               ✓
 *   §6.3  CTC: Eq (6.5) M·(q̈_cmd + K_jP·e_q + K_jD·ė_q)          ✓
 *          → C·q̇ + g + τ̂_F 在 Gazebo 中省略（模型失配）
 *   Alg.1  q_cmd 独立积分（非每步复位）                             ✓
 *   Alg.1  γ_a = 0.05                                               ✓
 */

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>
#include <franka/robot_state.h>

#include "../include/franka_model.h"

// ─────────────────────────────────────────────────────────────────────────────
using Vec7  = Eigen::Matrix<double, 7, 1>;
using Vec6  = Eigen::Matrix<double, 6, 1>;
using Mat7  = Eigen::Matrix<double, 7, 7>;
using Mat6  = Eigen::Matrix<double, 6, 6>;
using Mat67 = Eigen::Matrix<double, 6, 7>;
using Mat76 = Eigen::Matrix<double, 7, 6>;

namespace {

// ════════════════════════════════════════════════════════════
// Params (§4.2.3 公开设置表 + Algorithm 1 Requires)
// ════════════════════════════════════════════════════════════
struct Params {
    // GMO (§2, Eq 2.9)
    Vec7 K_O = (Vec7() << 40, 40, 40, 30, 25, 20, 20).finished();

    // FDI (§3.1, Eq 3.2)
    Vec7 tau_th_base = (Vec7() << 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3).finished();
    double kappa{0.1}, eps_fdi{0.02};

    // SVD投影噪声上界 (§3.3, Eq 3.7a + §6.2 共用)
    Vec7 r_noise = (Vec7() << 0.5, 0.5, 0.5, 0.5, 0.3, 0.3, 0.3).finished();
    double alpha_eps{1e-6};

    // DLS阻尼范围 (§4.2.3)
    double lambda2_base{1e-4}, lambda2_max{0.1};

    // sigmoid权重参数 (§4.2.3, Eq 4.12-4.13)
    double w0{1.0}, c0{1.0}, beta_w{10.0}, beta_c{10.0};

    // 非对称EMA (§4.5, Eq 4.19a)
    double gamma_attack{0.3}, gamma_decay{0.02};

    // CLIK P增益 (§6.1, Eq 6.1c)
    Vec6 K_P = (Vec6() << 30, 30, 30, 15, 15, 15).finished();

    // 零空间柔顺 (§6.2, Eq 6.2a–6.2b)
    Vec7 K_n = (Vec7() << 5, 5, 5, 5, 4, 3, 3).finished();
    double gamma_n{0.05};

    // CTC 关节PD + 加速度EMA (§6.3, Algorithm 1)
    Vec7 K_jP = (Vec7() << 80, 80, 80, 60, 40, 30, 20).finished();
    Vec7 K_jD = (Vec7() << 8,  8,  8,  6,  4,  3,  2 ).finished();
    double gamma_a{0.05};  // Alg.1: 0.05

    // v5.12: §6.1.1 笛卡尔力驱动β参数 (Eq 6.1a-2)
    double F_th{2.0};    // [N] 各笛卡尔方向力噪声阈值
    double F_max{40.0};  // [N] 满量程碰撞力 (β=1时对应力值)

    // v6.1: 有界柔顺 — 各笛卡尔方向最大自由退让量 (位置 m / 姿态 rad)
    Vec6 delta_budget = (Vec6() << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished();

    // 安全限幅
    double delta_tau_max{10.0};

    static Params forMethod(const std::string& m) {
        Params p;
        if (m == "M1") { p.lambda2_max = 1e-4; p.K_n.setZero(); }
        else if (m == "M2") { p.lambda2_base = 0.5; p.lambda2_max = 0.5; }
        else if (m == "M3") { p.lambda2_max = 1e-4; }
        return p;
    }
};

inline double sigmoid(double x) {
    return 1.0 / (1.0 + std::exp(-std::max(-50.0, std::min(50.0, x))));
}

// ════════════════════════════════════════════════════════════
// MODULE 2: FDI — 碰撞连杆隔离 (§3.1, Eq 3.2–3.2a)
// ════════════════════════════════════════════════════════════
struct FDIOut { Vec7 r_tilde, m_aff; int k_star; };

FDIOut fdi_isolate(const Vec7& r, const Vec7& tau_F, const Vec7& tau_m,
                   const Params& p) {
    Vec7 th = p.tau_th_base + p.kappa * tau_F.cwiseAbs()
                                + p.eps_fdi * tau_m.cwiseAbs();
    FDIOut o; o.r_tilde.setZero(); o.m_aff.setZero(); o.k_star = -1;
    for (int j = 6; j >= 0; --j)
        if (std::abs(r[j]) > th[j]) { o.k_star = j; break; }
    if (o.k_star >= 0)
        for (int j = 0; j <= o.k_star; ++j) o.m_aff[j] = 1.0;
    o.r_tilde = r.cwiseProduct(o.m_aff);
    return o;
}

// ════════════════════════════════════════════════════════════
// MODULE 3: SVD 投影 (§3.2–3.3, Eq 3.3, 3.6, 3.7, 3.7a)
// ════════════════════════════════════════════════════════════
struct SVDOut { Vec6 alpha, alpha_raw, alpha_th; Mat6 U; Vec6 s;
               Eigen::Matrix<double, 7, 6> V1; };

class SVDProjector {
public:
    explicit SVDProjector(const Params& p)
        : lb_(p.lambda2_base), rn_(p.r_noise), eps_(p.alpha_eps) {}

    SVDOut project(const Mat67& J, const Vec7& r_tilde) {
        SVDOut o;
        Eigen::JacobiSVD<Mat67> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        o.U = svd.matrixU(); o.s = svd.singularValues();
        o.V1 = svd.matrixV().leftCols(6);

        // 策略B: V1 符号追踪 (§4.5)
        if (Vp_.cols() > 0)
            for (int k = 0; k < 6; ++k)
                if (o.V1.col(k).dot(Vp_.col(k)) < 0) {
                    o.V1.col(k) *= -1; o.U.col(k) *= -1;
                }
        Vp_ = o.V1;

        for (int k = 0; k < 6; ++k) {
            double sk = o.s[k], co = sk / (sk*sk + lb_);

            // Eq 3.6: α_raw,k = σ_k/(σ_k²+λ²_base)·v_k^T·r̃
            o.alpha_raw[k] = co * o.V1.col(k).dot(r_tilde);

            // Eq 3.7a: α_th,k（不含 m_aff — 论文 v5.7 引入此乘子后
            //   k*=-1 时 α_th→ε, sigmoid(-β·ε)≈0.5 → 永久虚假阻尼。
            //   此处修正：所有关节噪声始终参与阈值计算）
            double ns = 0.0;
            for (int j = 0; j < 7; ++j) ns += std::abs(o.V1(j,k)) * rn_[j];
            o.alpha_th[k] = co * ns + eps_;
        }

        // Eq 3.7: 死区
        for (int k = 0; k < 6; ++k) {
            double a = o.alpha_raw[k], th = o.alpha_th[k];
            o.alpha[k] = (std::abs(a) >= th) ? (a - std::copysign(th, a)) : 0.0;
        }
        return o;
    }
private:
    double lb_; Vec7 rn_; double eps_;
    Eigen::Matrix<double, 7, 6> Vp_;
};

// ════════════════════════════════════════════════════════════
// MODULE 4: 笛卡尔基各向异性 DLS 阻尼矩阵 (§4 v6.0, Eq 4.9/4.12/4.13/4.20/4.21)
//   阻尼直接定义在笛卡尔坐标: Λ_cart = diag(λ²_X..λ²_RZ), 每个 λ²_i 由对应
//   笛卡尔力分量 |F_i| 经变分优化独立驱动 → 根除 UΛUᵀ 几何纠缠。
// ════════════════════════════════════════════════════════════
class ADLSDamping {
public:
    explicit ADLSDamping(const Params& p)
        : lb_(p.lambda2_base), lm_(p.lambda2_max),
          w0_(p.w0), c0_(p.c0), bw_(p.beta_w), bc_(p.beta_c),
          ga_(p.gamma_attack), gd_(p.gamma_decay), fth_(p.F_th) {}
    void reset() { lp_.setZero(); init_ = false; }

    /// 笛卡尔力旋量 F → 对角阻尼矩阵 Λ_cart (§4 v6.0)
    Mat6 compute_cart(const Vec6& F) {
        for (int i = 0; i < 6; ++i) {
            double cen = std::abs(F[i]) - fth_;              // |F_i| - F_th,i
            double wi  = w0_ * (1.0 - sigmoid(bw_ * cen)) + 1e-6;  // Eq 4.12
            double ci  = c0_ * sigmoid(bc_ * cen) + 1e-6;         // Eq 4.13
            lam2_[i] = (wi * lb_ + ci * lm_) / (wi + ci);    // Eq 4.9
        }
        // 逐元素非对称 EMA (Eq 4.20/4.21) — 无 UΛUᵀ, 无非对角项
        if (!init_) { lp_ = lam2_; init_ = true; }
        else for (int i = 0; i < 6; ++i) {
            double g = (lam2_[i] > lp_[i]) ? ga_ : gd_;
            lp_[i] = (1 - g) * lp_[i] + g * lam2_[i];
        }
        return lp_.asDiagonal();
    }

    Vec6 debug_lam2() const { return lp_; }

private:
    double lb_, lm_, w0_, c0_, bw_, bc_, ga_, gd_, fth_;
    Vec6 lam2_{Vec6::Zero()};
    Vec6 lp_{Vec6::Zero()};       // 逐方向 EMA 状态
    bool init_{false};
};

// ════════════════════════════════════════════════════════════
// MODULE 5: 伪逆 (§5, M1/M2/M3/M4)
// ════════════════════════════════════════════════════════════
struct PinvOut { Mat76 J_plus; Mat7 N; };

PinvOut compute_pinv(const Mat67& J, const Mat6& Lc, const std::string& method) {
    PinvOut o;
    if (method == "M1" || method == "M3") {
        Eigen::JacobiSVD<Mat67> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        auto s = svd.singularValues();
        Mat6 Si = Mat6::Zero();
        for (int k = 0; k < 6; ++k) if (s[k] > 1e-10) Si(k,k) = 1.0/s[k];
        o.J_plus.noalias() = svd.matrixV().leftCols(6) * Si
                             * svd.matrixU().transpose();
    } else {
        // Eq 4.4: J⁺ = J^T (J J^T + Λ)^(-1)
        o.J_plus.noalias() = J.transpose()
                           * (J * J.transpose() + Lc).inverse();
    }
    o.N = Mat7::Identity() - o.J_plus * J;
    return o;
}

// ════════════════════════════════════════════════════════════
// MODULE 6: 零空间柔顺 (§6.2, Eq 6.2–6.2b)
//   论文规定与 §3.3 共用 r_noise。
//   Gazebo 注: 实际模型失配 ~2-5 N·m, 若 r_noise=0.5 会导致
//   零空间持续漂移。此处保留论文设计，如 Gazebo 中需要可
//   通过 YAML 覆盖 r_noise 参数。
// ════════════════════════════════════════════════════════════
class NullCompliance {
public:
    explicit NullCompliance(const Params& p)
        : Kni_(p.K_n.cwiseInverse().asDiagonal()),
          dz_(p.r_noise), gn_(p.gamma_n) {}   // 共用 r_noise (§6.2)
    void reset() { qp_.setZero(); init_ = false; }

    Vec7 compute(const Vec7& r) {
        Vec7 rd = Vec7::Zero();
        for (int j = 0; j < 7; ++j)                      // Eq 6.2: 死区
            if (std::abs(r[j]) >= dz_[j])
                rd[j] = r[j] - std::copysign(dz_[j], r[j]);
        Vec7 raw = Kni_ * rd;                             // Eq 6.2a
        constexpr double QM = 0.3;
        raw = raw.cwiseMax(-QM).cwiseMin(QM);
        if (!init_) { qp_ = raw; init_ = true; return raw; }
        Vec7 qc = (1-gn_)*qp_ + gn_*raw;                 // Eq 6.2b: EMA
        qp_ = qc;
        return qc;
    }
private:
    Mat7 Kni_; Vec7 dz_; double gn_;
    Vec7 qp_{Vec7::Zero()}; bool init_{false};
};

}  // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
namespace variable_stiffness_controllers {

class ADLSController : public controller_interface::MultiInterfaceController<
    franka_hw::FrankaModelInterface,
    hardware_interface::EffortJointInterface,
    franka_hw::FrankaStateInterface> {

public:
    bool init(hardware_interface::RobotHW*, ros::NodeHandle&) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time&, const ros::Duration&) override;

private:
    Vec7 sat_tau(const Vec7& raw, const Vec7& prev);

    // 硬件接口
    std::unique_ptr<franka_hw::FrankaModelHandle> model_;
    std::unique_ptr<franka_hw::FrankaStateHandle>  state_;
    std::vector<hardware_interface::JointHandle>    joints_;

    // 算法模块 (v5.13: 外力矩直读, 不再使用 GMO)
    std::unique_ptr<SVDProjector>   svd_;
    std::unique_ptr<ADLSDamping>    damp_;
    std::unique_ptr<NullCompliance> null_;

    // 参数
    Params      p_;
    std::string method_{"M4"}, ema_mode_{"asymmetric"}, traj_mode_{"circle"};
    bool        ref_mod_{true};
    double      extend_dist_{0.5};

    // 时间 & 轨迹
    ros::Duration      t_;
    double             cnt_{0};
    Eigen::Vector3d    pos0_;
    Eigen::Quaterniond ori0_;
    bool               traj_on_{false};
    double             rad_{0.05}, omg_{2*M_PI/5.0};

    // CTC 状态 (Algorithm 1 完整状态)
    Vec7 qcmd_{Vec7::Zero()};   // 独立积分轨迹 (§6.3)
    Vec7 dqcp_{Vec7::Zero()};   // q̇_cmd 上一帧 (用于 q̈ 数值差分)
    Vec7 ddqe_{Vec7::Zero()};   // q̈_cmd EMA 状态
    Vec7 tprev_{Vec7::Zero()};  // τ_cmd 上一帧

    // 实验
    double fs_{3.0}, fe_{5.0}, fr_{0.2}, fm_{25.0};
    int    fd_{2};
    int    fd2_{-1};        // 第二施力方向(>=0 时与 fd_ 合成 45° 斜向力)
    Vec6   Fx_{Vec6::Zero()};
    franka::Frame contact_frame_{franka::Frame::kFlange};  // 接触连杆雅可比帧 (理想JTS真值)

    // 零空间次级任务 (姿态优化) — 用于验证两级任务分级
    bool   sec_task_{false};
    double Ksec_{1.5};
    Vec7   qpref_{Vec7::Zero()};

    ros::Publisher pub_;
};

// ═══════════════════════════════════════════════════════════════════════════════
bool ADLSController::init(hardware_interface::RobotHW* hw, ros::NodeHandle& nh) {
    std::string arm_id;
    if (!nh.getParam("arm_id", arm_id)) {
        ROS_ERROR("ADLS: arm_id missing"); return false;
    }
    std::vector<std::string> jn;
    if (!nh.getParam("joint_names", jn) || jn.size() != 7) {
        ROS_ERROR("ADLS: joint_names invalid"); return false;
    }

    auto* mi = hw->get<franka_hw::FrankaModelInterface>();
    if (!mi) { ROS_ERROR("ADLS: no ModelInterface"); return false; }
    try { model_.reset(new franka_hw::FrankaModelHandle(
            mi->getHandle(arm_id + "_model")));
    } catch (hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM("ADLS: model: " << e.what()); return false;
    }

    auto* si = hw->get<franka_hw::FrankaStateInterface>();
    if (!si) { ROS_ERROR("ADLS: no StateInterface"); return false; }
    try { state_.reset(new franka_hw::FrankaStateHandle(
            si->getHandle(arm_id + "_robot")));
    } catch (hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM("ADLS: state: " << e.what()); return false;
    }

    auto* ei = hw->get<hardware_interface::EffortJointInterface>();
    if (!ei) { ROS_ERROR("ADLS: no EffortInterface"); return false; }
    for (auto& n : jn) {
        try { joints_.push_back(ei->getHandle(n));
        } catch (hardware_interface::HardwareInterfaceException& e) {
            ROS_ERROR_STREAM("ADLS: joint " << n << ": " << e.what());
            return false;
        }
    }

    nh.param<std::string>("method", method_, "M4");
    p_ = Params::forMethod(method_);

    // YAML 参数覆盖
    { std::vector<double> v;
      if (nh.getParam("K_O", v) && v.size()==7) p_.K_O = Eigen::Map<Vec7>(v.data()); }
    nh.param("lambda2_base",  p_.lambda2_base,  p_.lambda2_base);
    nh.param("lambda2_max",   p_.lambda2_max,   p_.lambda2_max);
    nh.param("gamma_attack",  p_.gamma_attack,  p_.gamma_attack);
    nh.param("gamma_decay",   p_.gamma_decay,   p_.gamma_decay);
    nh.param("beta_w",        p_.beta_w,        p_.beta_w);
    nh.param("beta_c",        p_.beta_c,        p_.beta_c);
    nh.param("F_th",          p_.F_th,          p_.F_th);
    nh.param("F_max",         p_.F_max,         p_.F_max);
    { std::vector<double> v;
      if (nh.getParam("r_noise", v) && v.size()==7) p_.r_noise = Eigen::Map<Vec7>(v.data()); }
    { std::vector<double> v;
      if (nh.getParam("tau_th_base", v) && v.size()==7) p_.tau_th_base = Eigen::Map<Vec7>(v.data()); }
    { std::vector<double> v;  // v6.1 有界柔顺退让预算
      if (nh.getParam("delta_budget", v) && v.size()==6) p_.delta_budget = Eigen::Map<Vec6>(v.data()); }

    // Exp 2B ablation
    nh.param<std::string>("ema_mode", ema_mode_, "asymmetric");
    nh.param("ref_modulation", ref_mod_, true);
    if (ema_mode_ == "none")
        p_.gamma_attack = p_.gamma_decay = 1.0;
    else if (ema_mode_ == "symmetric")
        p_.gamma_attack = p_.gamma_decay = 0.01;

    nh.param<std::string>("traj_mode", traj_mode_, "circle");
    nh.param("extend_distance", extend_dist_, 0.5);
    nh.param("force_start_time",  fs_, fs_);
    nh.param("force_end_time",    fe_, fe_);
    nh.param("force_ramp_time",   fr_, fr_);
    nh.param("force_magnitude",   fm_, fm_);
    nh.param("force_direction",   fd_, fd_);
    nh.param("force_direction2",  fd2_, fd2_);
    nh.param("secondary_task",    sec_task_, sec_task_);   // 零空间次级姿态任务开关
    nh.param("secondary_gain",    Ksec_, Ksec_);
    // 接触连杆 → 雅可比帧 (linkN 取其远端关节帧, 使真值 τ_ext=J_c^T·F 的
    // 受扰关节范围与 FDI 拓扑一致: link N → 关节 1..N 受扰)
    { std::string flink; nh.param<std::string>("force_link", flink, "link7");
      int ln = (flink.size() > 4) ? std::atoi(flink.c_str() + 4) : 7;
      ln = std::max(1, std::min(7, ln));
      contact_frame_ = static_cast<franka::Frame>(
          static_cast<int>(franka::Frame::kJoint1) + ln); }
    { double cr=0.0, cp=5.0;
      nh.param("circle_radius", cr, cr);
      nh.param("circle_period", cp, cp);
      rad_ = cr; omg_ = 2.0*M_PI/cp; }

    svd_  = std::make_unique<SVDProjector>(p_);
    damp_ = std::make_unique<ADLSDamping>(p_);
    null_ = std::make_unique<NullCompliance>(p_);

    pub_ = nh.advertise<std_msgs::Float64MultiArray>("/adls/diagnostics", 10);

    ROS_INFO_STREAM("ADLS: init done method=" << method_ << " λ_max=" << p_.lambda2_max
        << " γ_a=" << p_.gamma_attack << " γ_d=" << p_.gamma_decay
        << " γ_acc=" << p_.gamma_a);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
void ADLSController::starting(const ros::Time&) {
    auto rs = state_->getRobotState();
    Eigen::Map<Vec7> q0(rs.q.data());
    qcmd_ = q0; dqcp_.setZero(); ddqe_.setZero(); tprev_.setZero();

    // 次级任务目标姿态: 起始构型 + 偏置(使零空间自运动可见, 不影响末端)
    qpref_ = q0;
    qpref_[0] += 0.6; qpref_[2] += 0.6;   // 关节1、3 偏置 → 肘部摆动

    Eigen::Affine3d T0(Eigen::Matrix4d::Map(rs.O_T_EE.data()));
    pos0_ = T0.translation(); ori0_ = Eigen::Quaterniond(T0.linear());

    t_ = ros::Duration(0); cnt_ = 0; traj_on_ = false;

    damp_->reset(); null_->reset();

    ROS_INFO("ADLS: started EE=[%.3f %.3f %.3f]", pos0_.x(), pos0_.y(), pos0_.z());
}

// ═══════════════════════════════════════════════════════════════════════════════
// update() — Algorithm 1 (1 kHz)
// ═══════════════════════════════════════════════════════════════════════════════
void ADLSController::update(const ros::Time&, const ros::Duration& per) {
    double dt = per.toSec(); t_ += per; cnt_ += dt; double tn = t_.toSec();

    // ── 机器人状态 ──
    auto rs = state_->getRobotState();
    Eigen::Map<Vec7> q(rs.q.data()), dq(rs.dq.data()), tJ(rs.tau_J.data());

    // ── 动力学 ──
    auto Ma = model_->getMass();
    auto Ca = model_->getCoriolis();
    auto ga = model_->getGravity();
    auto Ja = model_->getZeroJacobian(franka::Frame::kEndEffector);
    Eigen::Map<Mat7>  M(Ma.data());
    Eigen::Map<Vec7>  g(ga.data());
    Eigen::Map<Mat67> J(Ja.data());

    // ── 末端位姿 ──
    Eigen::Affine3d Tc(Eigen::Matrix4d::Map(rs.O_T_EE.data()));
    Eigen::Vector3d pc = Tc.translation();
    Eigen::Quaterniond oc(Tc.linear());

    // ── 期望轨迹 ──
    constexpr double TT = 2.0;  // 轨迹开始时刻
    Eigen::Vector3d pd;
    Vec6 dxd = Vec6::Zero();

    if (traj_mode_ == "extend" && tn >= TT) {
        if (!traj_on_) { pos0_ = pc; traj_on_ = true;
            ROS_INFO("ADLS: extend traj +X %.2fm", extend_dist_); }
        double te = tn - TT, pg = std::min(1.0, te/5.0),
               s  = pg*pg*(3-2*pg);
        pd.x() = pos0_.x() + extend_dist_ * s;
        pd.y() = pos0_.y(); pd.z() = pos0_.z();
        if (te < 5.0) dxd[0] = extend_dist_ * 6*pg*(1-pg)/5.0;
    } else if (rad_ > 1e-6 && tn >= TT) {
        if (!traj_on_) { pos0_ = pc; traj_on_ = true;
            ROS_INFO("ADLS: circle traj center [%.3f %.3f %.3f]",
                     pos0_.x(), pos0_.y(), pos0_.z()); }
        double tt = tn - TT, tau = std::min(1.0, tt/3.0),
               amp = tau*tau*(3-2*tau) * rad_,
               sc = std::sin(omg_*tt), cc = std::cos(omg_*tt);
        pd.x() = pos0_.x() + amp*sc;
        pd.y() = pos0_.y() + amp*cc;
        pd.z() = pos0_.z();
        dxd[0] =  omg_*amp*cc; dxd[1] = -omg_*amp*sc;
    } else {
        pd = pos0_;
    }

    // ── 笛卡尔误差 (四元数法) ──
    Vec6 err; err.head(3) = pd - pc;
    { Eigen::Quaterniond ot = ori0_;
      if (ot.coeffs().dot(oc.coeffs()) < 0) oc.coeffs() *= -1;
      Eigen::Quaterniond qe(oc.inverse() * ot);
      err.tail(3) = Tc.linear() * Eigen::Vector3d(qe.x(), qe.y(), qe.z()); }

    // ── 仿真外力 ──
    Fx_.setZero();
    if (tn >= fs_ && tn <= fe_) {
        double r = 1.0, el = tn-fs_, re = fe_-tn;
        if (el < fr_)             r = std::min(r, el/fr_);
        if (re < fr_ && re > 0.0) r = std::min(r, re/fr_);
        if (fd2_ >= 0) {                       // 45° 斜向: 力幅按 √2 拆到两轴
            Fx_[fd_]  = -fm_ * r / std::sqrt(2.0);
            Fx_[fd2_] = -fm_ * r / std::sqrt(2.0);
        } else {
            Fx_[fd_] = -fm_ * r;
        }
    }

    Vec7 tau_F = Friction(dq);

    // ═══════════════════════════════════════════
    // STEP 1: 外部力矩感知 (§2, v5.13) — 理想关节力矩传感器
    //   理论假设理想 JTS, 直接给出外部接触力矩 (式 2.3: τ_sens = τ_ext)。
    //   仿真中以真值 τ_ext = J_c^T·F_ext 模拟之 (J_c 为接触连杆雅可比,
    //   F_ext = Fx_ 为施加的世界系外力旋量)——完全无动力学/模型失配污染。
    //   注: franka_gazebo 的 tau_ext_hat_filtered 受控制器力矩指令约定影响
    //   (effort−command+gravity 会把省略的 C·q̇ 误计为外力矩), 不适用本控制器;
    //   实机部署时此处替换为标定好的 JTS / tau_ext_hat_filtered 读数。
    // ═══════════════════════════════════════════
    Vec7 r = Vec7::Zero();
    if (Fx_.cwiseAbs().maxCoeff() > 0.0) {
        auto Jc_a = model_->getZeroJacobian(contact_frame_);
        Eigen::Map<Mat67> Jc(Jc_a.data());
        r = Jc.transpose() * Fx_;
    }

    // ═══════════════════════════════════════════
    // STEP 2: FDI (§3.1)
    //   时间门控: 力施加前 (t < fs_) 强制无碰撞, 保证施力前不误报。
    // ═══════════════════════════════════════════
    auto fdi = fdi_isolate(r, tau_F, tJ, p_);
    if (tn < fs_) fdi.k_star = -1;  // 施力前: 强制无碰撞

    // ═══════════════════════════════════════════
    // STEP 3: SVD 投影 (§3.2–3.3)
    // ═══════════════════════════════════════════
    auto sv = svd_->project(J, fdi.r_tilde);

    // ═══════════════════════════════════════════
    // STEP 3b: 笛卡尔末端碰撞力 (§6.1.1, Eq 6.1a-1)
    //   F = J(JJᵀ + λ²_base I)⁻¹ r̃   → DLS 正则化笛卡尔力旋量。
    //   同时驱动 STEP 4 的阻尼 (λ²_i←|F_i|) 与 STEP 4b 的误差调制 (β←|F_i|)。
    // ═══════════════════════════════════════════
    Vec6 F_ext = Vec6::Zero();
    if (method_ == "M4" && fdi.k_star >= 0)
        F_ext = (J * J.transpose() + p_.lambda2_base * Mat6::Identity())
                    .llt().solve(J * fdi.r_tilde);

    // ═══════════════════════════════════════════
    // STEP 4: 笛卡尔基各向异性 DLS 阻尼 (§4 v6.0, Eq 4.4/4.9/4.12-4.13/4.20-4.21)
    //   Λ_cart = diag(λ²_X..λ²_RZ), λ²_i 由 |F_i| 经变分优化独立驱动。
    //   笛卡尔对角 → 天然方向选择性, 无 UΛUᵀ 几何纠缠。
    // ═══════════════════════════════════════════
    Mat6 Lc;
    if (method_ == "M2") {
        Lc = p_.lambda2_max * Mat6::Identity();
    } else if (method_ == "M1" || method_ == "M3") {
        Lc = p_.lambda2_base * Mat6::Identity();
    } else {
        Lc = damp_->compute_cart(F_ext);
    }

    // ═══════════════════════════════════════════
    // STEP 4b: 笛卡尔力驱动误差调制 — 有界柔顺 (§6.1.1 v6.1, Eq 6.1a-2/6.1b)
    //   β_i = clip((|F_i|-F_th)/(F_max-F_th), 0, 1)  → 逐方向碰撞参与度
    //   ẽ_eff = ẽ − clip(β⊙ẽ, −Δ_budget, +Δ_budget)
    //   自由退让量封顶 Δ_budget: 预算内 β→1 时完全自由退让(ẽ_eff=0);
    //   超预算后误差反馈重新接管(ẽ_eff=|ẽ|−Δ_budget>0)形成软刹车,
    //   消除持续外力下的无界退让→指令饱和/运动学耦合。
    // ═══════════════════════════════════════════
    Vec6 err_eff = err;
    if (method_ == "M4" && ref_mod_ && fdi.k_star >= 0) {
        Vec6 beta_cart;
        double rng = p_.F_max - p_.F_th;
        for (int i = 0; i < 6; ++i)
            beta_cart[i] = std::max(0.0, std::min(1.0,
                (std::abs(F_ext[i]) - p_.F_th) / rng));
        Vec6 mod = beta_cart.cwiseProduct(err);          // β⊙ẽ
        for (int i = 0; i < 6; ++i)                       // clip 至 ±Δ_budget
            mod[i] = std::max(-p_.delta_budget[i],
                              std::min(p_.delta_budget[i], mod[i]));
        err_eff = err - mod;
    }

    // ═══════════════════════════════════════════
    // STEP 5: 伪逆 + 零空间投影 (§5)
    // ═══════════════════════════════════════════
    auto pinv = compute_pinv(J, Lc, method_);

    // ═══════════════════════════════════════════
    // STEP 6: 速度合成 (§6.1, §6.2, Eq 6.4)
    // ═══════════════════════════════════════════
    Vec7 dq_task = pinv.J_plus * (dxd + p_.K_P.asDiagonal() * err_eff);

    //   零空间承载: ① 力驱动柔顺 dq_comp(碰撞后激活)② 次级姿态任务(始终激活)
    //   两级任务分级: 主任务(dq_task)在第一级, 这两项被 N 投影到零空间(第二级)
    Vec7 dq_null = Vec7::Zero();
    if (method_=="M3"||method_=="M4") {
        Vec7 dq_sub = Vec7::Zero();
        if (tn>fs_ && rad_>1e-6) dq_sub += null_->compute(r);   // 力驱动柔顺
        if (sec_task_)           dq_sub += Ksec_ * (qpref_ - q);// 次级姿态任务
        dq_null = pinv.N * dq_sub;
    }

    Vec7 dq_cmd = dq_task + dq_null;

    // v6.2: 逐关节独立限幅(取代均匀缩放)。均匀缩放 dq*=DQM/mx 会把高优先级
    //   任务方向(健康轴)的精确速度分量与受力方向的过大指令一起按比例砍;
    //   逐关节 clip 只截真正超限的关节,保留未超限关节里的任务跟踪分量。
    constexpr double DQM = 10.0;   // v6.2: 放开速度限幅(允许超Franka限以保X恢复)
    for (int j = 0; j < 7; ++j)
        dq_cmd[j] = std::max(-DQM, std::min(DQM, dq_cmd[j]));

    // ═══════════════════════════════════════════
    // STEP 7: CTC 力矩 (§6.3, Eq 6.5, Algorithm 1)
    //   τ = M·(q̈_cmd + K_jP·e_q + K_jD·ė_q) + [C·q̇ + g + τ̂_F]
    //
    //   方括号项: franka_hw_sim 自动补偿重力;
    //   C·q̇ 在慢速下 <0.1N·m, Gazebo 中 libfranka 分析模型
    //   与 ODE 物理引擎的 C 矩阵不一致, 加入反而引入误差。
    //   实机部署时取消注释。
    // ═══════════════════════════════════════════
    qcmd_ = qcmd_ + dq_cmd * dt;           // 独立积分 (非每步复位)
    Vec7 eq  = qcmd_ - q;
    // 安全限幅：防止 q_cmd 独立积分在碰撞失稳时漂移过大导致 CTC 饱和
    constexpr double EQ_MAX = 0.50;       // [rad] ≈ 2.9° — 足够覆盖正常跟踪误差
    for (int j = 0; j < 7; ++j)
        eq[j] = std::max(-EQ_MAX, std::min(EQ_MAX, eq[j]));
    Vec7 deq = dq_cmd - dq;

    // 加速度前馈: q̈_cmd 由 q̇_cmd 数值差分 + EMA 平滑
    Vec7 ddq_raw = (dq_cmd - dqcp_) / dt;  // numerical derivative
    dqcp_ = dq_cmd;
    ddqe_ = (1-p_.gamma_a)*ddqe_ + p_.gamma_a*ddq_raw; // EMA (§6.3)

    // CTC: M·(q̈_cmd + K_jP·e_q + K_jD·ė_q)
    Vec7 ddq_tot = ddqe_ + p_.K_jD.asDiagonal()*deq + p_.K_jP.asDiagonal()*eq;
    double ramp = std::min(1.0, tn/1.0);
    Vec7 tau_raw = ramp * (M * ddq_tot);
    // + C·q̇ + g + tau_F  — 实机取消注释

    Vec7 tau_cmd = sat_tau(tau_raw, tprev_);
    tprev_ = tau_cmd;

    for (size_t i = 0; i < 7; ++i) joints_[i].setCommand(tau_cmd[i]);

    ROS_INFO_THROTTLE(1.0,
        "[t=%.1fs %s] err=%.1fmm λ2=[%.4f,%.4f] r=%.1f k*=%d τ=%.2f",
        tn, (rad_<=1e-6?"STAT":(traj_on_?"CIRC":"HOLD")),
        1e3*err.head(3).norm(),
        damp_->debug_lam2().minCoeff(), damp_->debug_lam2().maxCoeff(),
        r.cwiseAbs().maxCoeff(), fdi.k_star, tau_cmd.cwiseAbs().maxCoeff());

    // ── 诊断 (100 Hz, 0-92 兼容现有脚本; 93-94 次级任务字段) ──
    constexpr double PDT = 0.01;
    if (cnt_ >= PDT) { cnt_ = 0;
        std_msgs::Float64MultiArray m; m.data.resize(95); int i = 0;
        m.data[i++]=tn;
        for(int k=0;k<6;++k) m.data[i++]=sv.alpha[k];       // 1-6
        for(int k=0;k<6;++k) m.data[i++]=damp_->debug_lam2()[k];//7-12
        for(int k=0;k<7;++k) m.data[i++]=r[k];               // 13-19
        for(int k=0;k<6;++k) m.data[i++]=err[k];             // 20-25
        m.data[i++]=(J*pinv.N).norm();                       // 26
        for(int k=0;k<6;++k) m.data[i++]=sv.s[k];            // 27-32
        for(int k=0;k<3;++k) m.data[i++]=pc[k];              // 33-35
        for(int k=0;k<3;++k) m.data[i++]=pd[k];              // 36-38
        for(int k=0;k<6;++k) m.data[i++]=dxd[k];             // 39-44
        for(int k=0;k<7;++k) m.data[i++]=dq[k];              // 45-51
        for(int k=0;k<7;++k) m.data[i++]=dq_cmd[k];          // 52-58
        for(int k=0;k<7;++k) m.data[i++]=tau_cmd[k];         // 59-65
        for(int k=0;k<7;++k) m.data[i++]=r[k];               // 66-72 (外力矩, v5.13 直读)
        for(int k=0;k<6;++k) m.data[i++]=Fx_[k];             // 73-78
        m.data[i++]=static_cast<double>(fdi.k_star);          // 79
        m.data[i++]=static_cast<double>(method_=="M1"?1:method_=="M2"?2:method_=="M3"?3:4);//80
        for(int k=0;k<6;++k) m.data[i++]=sv.alpha_raw[k];    // 81-86
        for(int k=0;k<6;++k) m.data[i++]=Lc(k,k);            // 87-92
        m.data[i++]=(qpref_-q).norm();                       // 93 次级任务代价‖q-q_pref‖
        m.data[i++]=dq_null.cwiseAbs().maxCoeff();            // 94 零空间速度峰
        pub_.publish(m);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
Vec7 ADLSController::sat_tau(const Vec7& raw, const Vec7& prev) {
    Vec7 o;
    for (int j = 0; j < 7; ++j) {
        double d = raw[j] - prev[j];
        d = std::max(-p_.delta_tau_max, std::min(p_.delta_tau_max, d));
        o[j] = prev[j] + d;
        o[j] = std::max(-(j<=3?30.0:8.0), std::min(j<=3?30.0:8.0, o[j]));
    }
    return o;
}

}  // namespace variable_stiffness_controllers

PLUGINLIB_EXPORT_CLASS(variable_stiffness_controllers::ADLSController,
                       controller_interface::ControllerBase)
