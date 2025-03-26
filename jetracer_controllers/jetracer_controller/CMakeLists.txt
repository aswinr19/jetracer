cmake_minimum_required(VERSION 3.8)
project(jetracer_controller)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(jetracer_controller
        src/JetracerController.cpp
        src/JetracerMock.cpp
        src/JetracerSerial.cpp
)
target_include_directories(jetracer_controller PUBLIC
        include
)
ament_target_dependencies(jetracer_controller rclcpp geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros)
target_link_libraries(jetracer_controller "${cpp_typesupport_target}")

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

install(TARGETS
        jetracer_controller
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        config/
        DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME})

ament_package()
