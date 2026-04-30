#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <vector>
#include <iostream>

int main() {
    // 1. 物理パラメータの設定 (Eigenを使用)
    double diameter = 0.067;      // ボールの直径 (m) - テニスボールくらい
    Eigen::Vector3d p0(0, 1.5, 0); // 初期位置 (x, y, z) = 床から1.5mの高さ
    Eigen::Vector3d v0(2.0, 5.0, 10.0); // 初期速度 (vx, vy, vz) = 前方に鋭く投げる
    Eigen::Vector3d g(0, -9.81, 0);    // 重力加速度

    // アニメーション用変数
    float t = 0.0f;
    float dt = 0.01f; // 1フレームあたりの進む時間 (sec)
    std::vector<Eigen::Vector3d> trajectory; // 軌跡保存用

    // 2. Pangolin ウィンドウ設定
    pangolin::CreateWindowAndBind("Ball Physics Sim", 1024, 768);
    glEnable(GL_DEPTH_TEST);

    // カメラ設定
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 420, 420, 512, 384, 0.1, 1000),
        pangolin::ModelViewLookAt(-5, 3, -10, 0, 1, 0, pangolin::AxisY)
    );

    // ハンドラー設定(マウス操作など)
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f/768.0f)
        .SetHandler(&handler);

    // 3. メインループ
    while(!pangolin::ShouldQuit()) {
        // 画面クリア
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        // カメラを適用
        d_cam.Activate(s_cam);

        // --- 物理演算 ---
        // 現在のボール位置を計算
        Eigen::Vector3d p = p0 + v0 * t + 0.5 * g * t * t;

        // 床（y=0）に着いたらリセットする
        if (p.y() < 0) {
            t = 0;
            trajectory.clear();
        } else {
            t += dt;
            trajectory.push_back(p);
        }

        // --- 描画ロジック ---
        
        // 1. 座標軸
        pangolin::glDrawAxis(0.5);

        // 2. 地面（グリッド）を自前で描画
        glColor3f(0.3f, 0.3f, 0.3f);
        glBegin(GL_LINES);
        for(int i=-10; i<=10; ++i) {
            glVertex3f(i*0.5f, 0, -5); glVertex3f(i*0.5f, 0, 5); // 縦線
            glVertex3f(-5, 0, i*0.5f); glVertex3f(5, 0, i*0.5f); // 横線
        }
        glEnd();

        // 3. 軌跡（線）
        glLineWidth(2);
        glColor3f(0.0f, 1.0f, 1.0f);
        glBegin(GL_LINE_STRIP);
        for(const auto& pos : trajectory) {
            glVertex3d(pos.x(), pos.y(), pos.z());
        }
        glEnd();

        // 4. ボール（簡易的な立方体、または点として描画）
        // OpenGLで綺麗な球を描くのは少し長いので、まずは「色付きの箱」で代用します
        glPushMatrix(); // 現在の行列（変換行列）を保存
        glTranslated(p.x(), p.y(), p.z()); // ボールの位置へ描画の原点を移動
        glColor3f(1.0f, 0.5f, 0.0f);
        pangolin::glDrawColouredCube(-diameter/2.0, diameter/2.0); // Pangolinにある標準的な箱
        glPopMatrix();  // 行列を元に戻す

        pangolin::FinishFrame();
    }

    return 0;
}
