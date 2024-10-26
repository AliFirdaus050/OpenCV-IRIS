cmake_minimum_required(VERSION 3.10)

# Nama proyek
project(MyOpenCVProject)

# Mencari OpenCV yang terinstal di sistem
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Ganti dengan path ke direktori Eigen Anda
include_directories("/home/ali/Eigen")

# Menambahkan executable dari main.cpp
add_executable(MyOpenCVProject Tugas1.cpp)

# Menautkan library OpenCV
target_link_libraries(MyOpenCVProject ${OpenCV_LIBS})