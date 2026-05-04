#pragma once
#include <Eigen/Dense>
#include <vector>


// 着弾予想の中心と分散をまとめた構造体
struct ImpactPrediction {
    Eigen::Vector2d pos;     // 予測位置
    Eigen::Matrix2d cov;     // 予測の不確かさ（共分散行列）
    bool reachable;
};

class BallEKF {
public:
    /**
     * @param initial_pos 初期位置の推定値
     * @param initial_vel 初期速度の推定値
     */
    BallEKF(const Eigen::Vector3d& initial_pos, const Eigen::Vector3d& initial_vel);

    // カメラの内部・外部パラメータをセットする
    void setCameraParams(double fx, double fy, double cx, double cy, 
                        const Eigen::Matrix3d& R, const Eigen::Vector3d& t);

    // 予測と更新を行うメイン関数
    void step(const Eigen::Vector2d& z, double dt, bool observed);

    // 現在の状態を取得
    Eigen::VectorXd state() const { return x_; }
    
    // スクリーン(z=5.0)への到達予測地点を計算
    ImpactPrediction predictScreenImpact(double screen_z = 5.0) const;

private:
    // 状態ベクトル x = [x, y, z, vx, vy, vz]^T
    Eigen::VectorXd x_;
    // 誤差共分散行列
    Eigen::MatrixXd P_;
    // プロセスノイズ Q, 観測ノイズ R
    Eigen::MatrixXd Q_, R_noise_;

    // カメラパラメータ
    double fx_, fy_, cx_, cy_;
    Eigen::Matrix3d R_cam_; // 世界座標 -> カメラ座標の回転
    Eigen::Vector3d t_cam_; // 世界座標 -> カメラ座標の並進

    // 内部計算用
    void predict(double dt);
    void update(const Eigen::Vector2d& z);
};
