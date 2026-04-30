# 1. CMakeの最低バージョン
cmake_minimum_required(VERSION 3.10)
project(BallTrackerSim)

# Homebrewのインストール先を検索パスに加える
list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")

# 2. C++のバージョン指定
set(CMAKE_CXX_STANDARD 17)

# 3. 外部ライブラリを探索
set(Pangolin_DIR "/opt/homebrew/lib/cmake/Pangolin")
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# 4. 実行ファイルを作る
add_executable(ball_sim main.cpp)

# 5. ライブラリとパスを紐付ける（変数形式に変更）
target_link_libraries(ball_sim 
    ${Pangolin_LIBRARIES} 
    ${OpenCV_LIBS} 
    Eigen3::Eigen
)

# ヘッダーファイルの場所をコンパイラに明示的に教える
target_include_directories(ball_sim PRIVATE 
    ${Pangolin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)
