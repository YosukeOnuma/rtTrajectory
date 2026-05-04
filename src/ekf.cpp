#include "ekf.hpp"
#include <iostream>

BallEKF::BallEKF(const Eigen::Vector3d& initial_pos, const Eigen::Vector3d& initial_vel) {
    x_ = Eigen::VectorXd(6);
    x_ << initial_pos, initial_vel;

    // 初期自信（P）: 最初は少しあやふや
    P_ = Eigen::MatrixXd::Identity(6, 6) * 1.0;

    // プロセスノイズ（Q）: 物理モデルの不確かさ（空気抵抗の変動など）
    Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.01;

    // 観測ノイズ（R）: カメラの測定誤差（ピクセル単位）
    R_noise_ = Eigen::MatrixXd::Identity(2, 2) * 2.0; 
}

void BallEKF::setCameraParams(double fx, double fy, double cx, double cy, 
                             const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
    fx_ = fx; fy_ = fy; cx_ = cx; cy_ = cy;
    R_cam_ = R; t_cam_ = t;
}

void BallEKF::step(const Eigen::Vector2d& z, double dt, bool observed) {
    predict(dt);    // 状態発展の予測ステップ
    if (observed) {
        update(z);  // 観測をもとに状態を修正する更新ステップ
    }
}

void BallEKF::predict(double dt) {
    double g = -9.81;

    // 1. 状態の遷移 (f(x))
    Eigen::VectorXd x_new = x_;
    x_new(0) += x_(3) * dt; // x += vx*dt
    x_new(1) += x_(4) * dt + 0.5 * g * dt * dt; // y += vy*dt + 0.5*g*dt^2
    x_new(2) += x_(5) * dt; // z += vz*dt
    x_new(3) += 0;          // vx
    x_new(4) += g * dt;     // vy += g*dt
    x_new(5) += 0;          // vz
    x_ = x_new;

    // 2. ヤコビアン F = df/dx（EKFの線形近似）
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;

    // 3. 誤差共分散の更新 P = FPF^T + Q
    P_ = F * P_ * F.transpose() + Q_;
}

void BallEKF::update(const Eigen::Vector2d& z) {
    // --- 観測モデル h(x) の計算 ---
    // 推定している状態 x_ から、ピクセル座標での見え方の推定を計算する
    Eigen::Vector3d p_world = x_.head<3>();
    Eigen::Vector3d p_cam = R_cam_ * p_world + t_cam_;

    double Xc = p_cam.x();
    double Yc = p_cam.y();
    double Zc = p_cam.z();

    if (Zc <= 0.1) return; // カメラの真横や後ろにある時は無視

    Eigen::Vector2d z_pred;
    z_pred << fx_ * (Xc / Zc) + cx_, fy_ * (Yc / Zc) + cy_;     // これが観測モデル h(x) の出力

    // --- 観測のヤコビアン H = dh/dx の計算 ---
    // 連鎖律: dh/dx = (dh/dp_cam) * (dp_cam/dp_world) * (dp_world/dx)
    
    // dh/dp_cam (2x3)
    Eigen::MatrixXd J_proj(2, 3);
    J_proj << fx_/Zc,    0, -fx_*Xc/(Zc*Zc),
                0, fy_/Zc, -fy_*Yc/(Zc*Zc);

    // dp_cam/dp_world = R_cam_ (3x3)
    // dp_world/dx = [I(3x3) | 0(3x3)] (3x6)
    
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 6);    
    H.block<2, 3>(0, 0) = J_proj * R_cam_;

    // --- カルマンゲイン K の計算 ---
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_noise_;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    // --- 状態 x と 自信 P の修正 ---
    x_ = x_ + K * (z - z_pred);
    P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_;
}

ImpactPrediction BallEKF::predictScreenImpact(double screen_z) const {
    ImpactPrediction res;
    double x = x_(0), y = x_(1), z = x_(2);
    double vx = x_(3), vy = x_(4), vz = x_(5);
    double g = -9.81;

    if (vz <= 0.1 || z >= screen_z) {
        res.reachable = false;
        return res;
    }

    double dt = (screen_z - z) / vz;
    res.pos << x + vx * dt, y + vy * dt + 0.5 * g * dt * dt;
    res.reachable = true;

    // --- ヤコビアン Jg の計算 ---
    Eigen::MatrixXd Jg = Eigen::MatrixXd::Zero(2, 6);
    double vy_final = vy + g * dt; // 着弾時のy方向速度

    // dx_s / dx...
    Jg(0, 0) = 1.0;                  // dx_s / dx
    Jg(0, 2) = -vx / vz;            // dx_s / dz
    Jg(0, 3) = dt;                   // dx_s / dvx
    Jg(0, 5) = -vx * dt / vz;        // dx_s / dvz

    // dy_s / dy...
    Jg(1, 1) = 1.0;                  // dy_s / dy
    Jg(1, 2) = -vy_final / vz;       // dy_s / dz
    Jg(1, 4) = dt;                   // dy_s / dvy
    Jg(1, 5) = -vy_final * dt / vz;  // dy_s / dvz

    // 不確かさの伝搬 P_impact = Jg * P * Jg^T
    res.cov = Jg * P_ * Jg.transpose();

    return res;
}
