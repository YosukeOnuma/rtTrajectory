#include <pangolin/pangolin.h>
#include <Eigen/Core>

int main() {
    // 1. ウィンドウの作成
    pangolin::CreateWindowAndBind("Ball Tracker Sim", 1024, 768);
    glEnable(GL_DEPTH_TEST);

    // 2. 3Dビュー（カメラ）の設定
    // 引数: カメラ位置, 注視点, 上方向ベクトル
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 420, 420, 512, 384, 0.1, 1000),
        pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY)
    );

    // 3. マウス操作をハンドルするビューアーを作成
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f/768.0f)
        .SetHandler(&handler);

    while(!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 描画開始
        d_cam.Activate(s_cam);
        
        // --- ここに描画ロジックを書く ---

        // 座標軸を描いてみる
        pangolin::glDrawAxis(0.5);

        // 軌跡に見立てた適当な線
        glLineWidth(2);
        glColor3f(1.0f, 0.0f, 0.0f);
        glBegin(GL_LINE_STRIP);
        for(float t=0; t<2.0; t+=0.1) {
            glVertex3f(t, 1.0f - 0.5f*9.8f*t*t*0.1f, t); // 適当な放物線
        }
        glEnd();

        // ---------------------------

        pangolin::FinishFrame();
    }

    return 0;
}
