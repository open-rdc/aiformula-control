cmake_minimum_required(VERSION 3.8)
project(socketcan_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(socketcan_interface_node
        src/socketcan_interface_node.cpp
)

ament_target_dependencies(socketcan_interface_node
        ament_index_cpp
)

ament_auto_package(
        INSTALL_TO_SHARE config
)
