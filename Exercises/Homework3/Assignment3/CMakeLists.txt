cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_STANDARD 17)

set(Eigen3_DIR /opt/homebrew/Cellar/eigen/3.4.0_1/share/eigen3/cmake)

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp Texture.hpp Texture.cpp Shader.hpp OBJ_Loader.h)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})
target_link_libraries(Rasterizer Eigen3::Eigen)
#target_compile_options(Rasterizer PUBLIC -Wall -Wextra -pedantic)