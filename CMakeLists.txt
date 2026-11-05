cmake_minimum_required(VERSION 3.22)

project(TipSim VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(mujoco CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_library(TINYXML2_LIB NAMES tinyxml2 REQUIRED)

set(TIPSIM_LIB_DIR "${CMAKE_SOURCE_DIR}/lib")

function(tipsim_add_sample name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE 
    -Wl,--no-as-needed ${TINYXML2_LIB} -Wl,--as-needed 
    mujoco::mujoco glfw rclcpp::rclcpp sensor_msgs::sensor_msgs_library)
    target_include_directories(${name} PRIVATE ${CMAKE_SOURCE_DIR}/samples/common)
    target_compile_definitions(${name} PRIVATE
        TIP_REPO_ROOT="${CMAKE_SOURCE_DIR}"
        TIP_LIB_DIR="${TIPSIM_LIB_DIR}"
    )
endfunction()

tipsim_add_sample(rh56e2_hold_cylinder samples/inspire/cpp/rh56e2_hold_cylinder.cpp)
tipsim_add_sample(rh56e2_grasp_jelly samples/inspire/cpp/rh56e2_grasp_jelly.cpp)
tipsim_add_sample(loong96 samples/tip/cpp/loong96.cpp)
tipsim_add_sample(universa_finger samples/tip/cpp/universa_finger.cpp)
tipsim_add_sample(universa_pick_and_place samples/tip/cpp/universa_pick_and_place.cpp)
tipsim_add_sample(universa_grasp_jelly samples/tip/cpp/universa_grasp_jelly.cpp)