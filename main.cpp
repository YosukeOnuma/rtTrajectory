#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <vector>
#include <cstdlib>


int main() {
    // 1. 物理パラメータ
    double diameter = 0.067;
    Eigen::Vector3d p0(0, 1.5, 0);
    Eigen::Vector3d v0(0.0, 5.0, 10.0);
    Eigen::Vector3d g(0, -9.81, 0);

    float t = 0.0f;
    float dt = 0.005f;
    std::vector<Eigen::Vector3d> trajectory;

    // 2. ウィンドウ設定 (横長に設定: 1500x600)
    pangolin::CreateWindowAndBind("Ball Tracker: Dual View", 1500, 600);
    glEnable(GL_DEPTH_TEST);

    // --- カメラ1 (左: 自由視点) ---
    pangolin::OpenGlRenderState s_cam1(
        pangolin::ProjectionMatrix(750, 600, 420, 420, 375, 300, 0.1, 1000),
        pangolin::ModelViewLookAt(-2.5, 3, -5, 0, 1, 0, pangolin::AxisY)
    );
    pangolin::Handler3D handler1(s_cam1);

    // --- カメラ2 (右: 固定視点) ---
    // 位置(0, 4, 5), 注視点(0, 1, 0)
    pangolin::OpenGlRenderState s_cam2(
        pangolin::ProjectionMatrix(750, 600, 420, 420, 375, 300, 0.1, 1000),
        pangolin::ModelViewLookAt(0, 4, 5, 0, 1, 0, pangolin::AxisY)
    );
    // 固定視点にするため、handlerは無し（または操作不能にする）

    // --- Viewのレイアウト設定 ---
    // SetBounds(bottom, top, left, right) ※0.0〜1.0の相対座標
    pangolin::View& d_cam1 = pangolin::Display("cam1")
        .SetBounds(0.0, 1.0, 0.0, 0.5, -750.0f/600.0f)
        .SetHandler(&handler1);

    pangolin::View& d_cam2 = pangolin::Display("cam2")
        .SetBounds(0.0, 1.0, 0.5, 1.0, -750.0f/600.0f);

    // 3. メインループ
    while(!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // --- 物理演算 ---
        Eigen::Vector3d p = p0 + v0 * t + 0.5 * g * t * t;
        if (p.y() < 0) {
            t = 0; trajectory.clear();
            double r = (double)rand() / RAND_MAX;
            // -1~1に変換
            r = r * 2.0 - 1.0;
            v0 = Eigen::Vector3d(0 + 1.3 * r, 5.5 + 2.0 * r, 10.0 + 1.0 * r);
        } else {
            t += dt; trajectory.push_back(p);
        }

        // --- 共通の描画ロジックをラムダ式で定義 ---
        auto draw_scene = [&]() {
            pangolin::glDrawAxis(0.5);
            
            // 地面
            glColor3f(0.3f, 0.3f, 0.3f);
            glBegin(GL_LINES);
            for(int i=-10; i<=10; ++i) {
                glVertex3f(i*0.5f, 0, -5); glVertex3f(i*0.5f, 0, 5);
                glVertex3f(-5, 0, i*0.5f); glVertex3f(5, 0, i*0.5f);
            }
            glEnd();

            // スクリーン
            glColor3f(1.0f, 1.0f, 1.0f);
            glBegin(GL_LINE_LOOP);
            glVertex3f(-1.6f, 2.0f, 5.0f); glVertex3f(1.6f, 2.0f, 5.0f);
            glVertex3f(1.6f, 3.8f, 5.0f); glVertex3f(-1.6f, 3.8f, 5.0f);
            glEnd();

            // 軌跡
            glLineWidth(2);
            glColor3f(0.0f, 1.0f, 1.0f);
            glBegin(GL_LINE_STRIP);
            for(const auto& pos : trajectory) glVertex3d(pos.x(), pos.y(), pos.z());
            glEnd();

            // ボール
            glPushMatrix();
            glTranslated(p.x(), p.y(), p.z());
            glColor3f(1.0f, 0.5f, 0.0f);
            pangolin::glDrawColouredCube(-diameter/2.0, diameter/2.0);
            glPopMatrix();
        };

        // --- 左側の描画 ---
        d_cam1.Activate(s_cam1);
        draw_scene();

        // --- 右側の描画 ---
        d_cam2.Activate(s_cam2);
        draw_scene();

        pangolin::FinishFrame();
    }

    return 0;
}
