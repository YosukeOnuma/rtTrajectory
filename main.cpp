#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>
#include <vector>
#include <cstdlib>
#include <iostream>
#include "ekf.hpp"

// 楕円を描画するための補助関数
void drawErrorEllipse(const Eigen::Vector2d& pos, const Eigen::Matrix2d& cov, float screen_z) {
    // 共分散行列から固有値と固有ベクトルを求め、楕円の形状を決定する
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
    Eigen::Vector2d eigenvalues = solver.eigenvalues();
    Eigen::Matrix2d eigenvectors = solver.eigenvectors();

    // 2シグマ（約95%信頼区間）の半径に設定
    float scale = 2.0f;
    float rx = scale * std::sqrt(std::max(0.0, eigenvalues[1])); 
    float ry = scale * std::sqrt(std::max(0.0, eigenvalues[0]));
    float angle = std::atan2(eigenvectors(1, 1), eigenvectors(0, 1));

    glPushMatrix();
    glTranslated(pos.x(), pos.y(), screen_z);
    glRotated(angle * 180.0 / M_PI, 0, 0, 1);
    
    glBegin(GL_LINE_LOOP);
    for(int i=0; i<36; ++i) {
        float theta = i * 10.0f * M_PI / 180.0f;
        glVertex3f(rx * std::cos(theta), ry * std::sin(theta), 0);
    }
    glEnd();
    glPopMatrix();
}

std::vector<double> generateRandomBallInit() {
    std::vector<double> init = {0.0, 1.5, 0.0, 0.0, 2.5, 10.0};
    std::default_random_engine generator(std::random_device{}()); 
    std::normal_distribution<double> distribution(0.0, 0.4);
    for (int i = 0; i < 6; ++i) {
        init[i] += distribution(generator);
    }
    return init;
}

int main() {
    //// 1. パラメータ
    //ボール 
    double diameter = 0.067;
    // define p0 and v0 with generateRandomBallInit(
    std::vector<double> init = generateRandomBallInit();
    Eigen::Vector3d p0(init[0], init[1], init[2]);
    Eigen::Vector3d v0(init[3], init[4], init[5]);
    std::cout << "Ball Initial position: " << p0.transpose() << std::endl;
    Eigen::Vector3d g_vec(0, -9.81, 0);

    // 時間
    float t_sim = 0.0f;     // シミュレーション時間
    constexpr float dt = 0.005f; // 物理演算周期 
    constexpr float CAMERA_UPDATE_DT = 1.0f / 60.0f; // カメラ観測周期 (60fps)
    float last_obs_t = 0.0f;
    std::vector<Eigen::Vector3d> trajectory_true;
    std::vector<Eigen::Vector3d> trajectory_ekf;
    std::vector<Eigen::Vector2d> trajectory_image_observed;

    // ワールド
    constexpr float SCREEN_X = 0.0f, SCREEN_Y = 1.0f; // スクリーンの中心座標
    constexpr float SCREEN_WIDTH = 4.0f, SCREEN_HEIGHT = 2.0f; // スクリーンの幅と高さ
    constexpr float SCREEN_Z = 5.0f; // スクリーンのz座標
    
    // カメラパラメータ
    constexpr double IMAGE_WIDTH = 750, IMAGE_HEIGHT = 600;
    constexpr double FX = 420, FY = 420;
    const double CX = IMAGE_WIDTH / 2.0, CY = IMAGE_HEIGHT / 2.0;


    // 2. EKFの初期化 (初期位置は真値に少しノイズを混ぜて開始)
    std::vector<double> initEKF = generateRandomBallInit();
    BallEKF ekf(Eigen::Vector3d(initEKF[0], initEKF[1], initEKF[2]), Eigen::Vector3d(initEKF[3], initEKF[4], initEKF[5]));
    std::cout << "EKF Initial position: " << ekf.state().head<3>().transpose() << std::endl;

    // 3. ウィンドウ設定
    pangolin::CreateWindowAndBind("Ball Tracker: EKF Prediction", 1500, 600);
    glEnable(GL_DEPTH_TEST);

    pangolin::OpenGlRenderState s_cam1(
        pangolin::ProjectionMatrix(IMAGE_WIDTH, IMAGE_HEIGHT, FX, FY, CX, CY, 0.1, 1000),
        pangolin::ModelViewLookAt(-2.5, 3, -5, 0, 1, 0, pangolin::AxisY)
    );
    pangolin::Handler3D handler1(s_cam1);

    // 第2カメラ（観測用）
    pangolin::OpenGlRenderState s_cam2(
        pangolin::ProjectionMatrix(IMAGE_WIDTH, IMAGE_HEIGHT, FX, FY, CX, CY, 0.1, 1000),
        pangolin::ModelViewLookAt(SCREEN_X, SCREEN_Y+SCREEN_HEIGHT/2+0.2, SCREEN_Z, 0, 1, 0, pangolin::AxisY)
    );

    // EKFにカメラパラメータを教える
    // PangolinのModelViewから R (回転) と t (並進) を抽出
    pangolin::OpenGlMatrix mv = s_cam2.GetModelViewMatrix();
    Eigen::Matrix3d R_cam;
    Eigen::Vector3d t_cam;
    for(int i=0; i<3; ++i) {
        for(int j=0; j<3; ++j) R_cam(i,j) = mv.m[j*4+i];
        t_cam(i) = mv.m[12+i];
    }
    ekf.setCameraParams(FX, FY, CX, CY, R_cam, t_cam);

    pangolin::View& d_cam1 = pangolin::Display("cam1").SetBounds(0.0, 1.0, 0.0, 0.5, -IMAGE_WIDTH/IMAGE_HEIGHT).SetHandler(&handler1);
    pangolin::View& d_cam2 = pangolin::Display("cam2").SetBounds(0.0, 1.0, 0.5, 1.0, -IMAGE_WIDTH/IMAGE_HEIGHT);

    while(!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // --- 物理演算 (真値) ---
        Eigen::Vector3d p_true = p0 + v0 * t_sim + 0.5 * g_vec * t_sim * t_sim;
        
        bool observed = false;
        Eigen::Vector2d z_uv(0, 0);

        if (p_true.y() < 0) { // リセット処理

            // 時間
            t_sim = 0; last_obs_t = 0.0f;
            // ボール
            std::vector<double> init = generateRandomBallInit();
            p0 = Eigen::Vector3d(init[0], init[1], init[2]);
            v0 = Eigen::Vector3d(init[3], init[4], init[5]);
            std::cout << "Ball Initial position: " << p0.transpose() << std::endl;
            trajectory_true.clear();
            // EKF
            std::vector<double> initEKF = generateRandomBallInit();
            ekf = BallEKF(Eigen::Vector3d(initEKF[0], initEKF[1], initEKF[2]), Eigen::Vector3d(initEKF[3], initEKF[4], initEKF[5])); // EKFもリセット
            ekf.setCameraParams(FX, FY, CX, CY, R_cam, t_cam);
            std::cout << "EKF Initial position: " << ekf.state().head<3>().transpose() << std::endl;
            trajectory_ekf.clear();
            trajectory_image_observed.clear();


        } else {
            // --- 60fps周期での観測生成 ---
            if (t_sim - last_obs_t >= CAMERA_UPDATE_DT && p_true.z() <= SCREEN_Z) {
                // 真値を画像上に投影
                Eigen::Vector3d p_c = R_cam * p_true + t_cam;
                z_uv << FX * (p_c.x() / p_c.z()) + CX, 
                        FY * (p_c.y() / p_c.z()) + CY;
                
                // 観測ノイズを付与 (標準偏差 2.0ピクセル程度)
                z_uv.x() += ((double)rand()/RAND_MAX - 0.5) * 2.0;
                z_uv.y() += ((double)rand()/RAND_MAX - 0.5) * 2.0;

                trajectory_image_observed.push_back(z_uv);
                // std::cout << "Time: " << t_sim << "s, Observed UV: " << z_uv.transpose() << std::endl;
                
                observed = true;
                last_obs_t = t_sim;
            }
            t_sim += dt;
            trajectory_true.push_back(p_true);
        }

        // --- EKFステップ実行 ---
        ekf.step(z_uv, dt, observed);
        trajectory_ekf.push_back(ekf.state().head<3>());
        ImpactPrediction pred = ekf.predictScreenImpact(SCREEN_Z);
        // std::cout << "Predicted Impact: X:" << pred.pos.x() << " Y:" << pred.pos.y() << " Reachable: " << pred.reachable << std::endl;

        // --- 描画ロジック ---
        auto draw_scene = [&](bool is_free_view) {
            pangolin::glDrawAxis(0.5);
            
            // 地面
            glColor3f(0.3f, 0.3f, 0.3f);
            glBegin(GL_LINES);
            for (int i = -10; i <= 10; ++i) {
                glVertex3f(i * 0.1f, 0.0f, -1.0f);
                glVertex3f(i * 0.1f, 0.0f,  1.0f);
                glVertex3f(-1.0f, 0.0f, i * 0.1f);
                glVertex3f( 1.0f, 0.0f, i * 0.1f);
            }
            glEnd();

            // スクリーン (白い線)
            glColor3f(1.0f, 1.0f, 1.0f);
            glBegin(GL_LINE_LOOP);
            glVertex3f(SCREEN_X - SCREEN_WIDTH/2.0f, SCREEN_Y + SCREEN_HEIGHT/2.0f, SCREEN_Z);
            glVertex3f(SCREEN_X + SCREEN_WIDTH/2.0f, SCREEN_Y + SCREEN_HEIGHT/2.0f, SCREEN_Z);
            glVertex3f(SCREEN_X + SCREEN_WIDTH/2.0f, SCREEN_Y - SCREEN_HEIGHT/2.0f, SCREEN_Z);
            glVertex3f(SCREEN_X - SCREEN_WIDTH/2.0f, SCREEN_Y - SCREEN_HEIGHT/2.0f, SCREEN_Z);
            glEnd();

            // 真値のボール (オレンジ)
            glPushMatrix();
            glTranslated(p_true.x(), p_true.y(), p_true.z());
            glColor3f(1.0f, 0.5f, 0.0f);
            pangolin::glDrawColouredCube(-diameter/2.0, diameter/2.0);
            glPopMatrix();

            // 真値の軌跡 (オレンジの線)
            glColor3f(1.0f, 0.5f, 0.0f);
            glBegin(GL_LINE_STRIP);
            for (const auto& pt : trajectory_true) {
                glVertex3d(pt.x(), pt.y(), pt.z());
            }
            glEnd();

            // EKFによる予測軌跡 (緑色の線)
            glColor3f(0.0f, 1.0f, 0.0f);
            glBegin(GL_LINE_STRIP);
            for (const auto& pt : trajectory_ekf) {
                glVertex3d(pt.x(), pt.y(), pt.z());
            }
            glEnd();

            // // EKF推定位置の表示 (緑色の小さな点)
            // Eigen::VectorXd x_est = ekf.state();
            // glColor3f(0.0f, 1.0f, 0.0f);
            // glPointSize(6.0f);
            // glBegin(GL_POINTS);
            // glVertex3d(x_est(0), x_est(1), x_est(2));
            // glEnd();

            // スクリーン上の着弾予測
            if (pred.reachable) {
                glColor3f(1.0f, 0.0f, 0.0f); // 赤色で描画
                // 予測地点に「+」マーク
                glBegin(GL_LINES);
                glVertex3f(pred.pos.x()-0.1, pred.pos.y(), SCREEN_Z); glVertex3f(pred.pos.x()+0.1, pred.pos.y(), SCREEN_Z);
                glVertex3f(pred.pos.x(), pred.pos.y()-0.1, SCREEN_Z); glVertex3f(pred.pos.x(), pred.pos.y()+0.1, SCREEN_Z);
                glEnd();
                // 不確かさの楕円を描画
                drawErrorEllipse(pred.pos, pred.cov, SCREEN_Z);
            }
        };

        d_cam1.Activate(s_cam1); draw_scene(true);
        d_cam2.Activate(s_cam2); draw_scene(false);

        pangolin::FinishFrame();
    }
    return 0;
}
