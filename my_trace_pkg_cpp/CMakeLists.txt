cmake_minimum_required(VERSION 3.8)
project(my_trace_pkg_cpp)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(opentelemetry-cpp_DIR /home/asus/backup/zzzzz/ros2/acceleration_robotics/trace_ws/dependencies/otel-cpp/lib/cmake/opentelemetry-cpp)

# Use find_package to include OpenTelemetry C++
find_package(opentelemetry-cpp CONFIG REQUIRED NO_DEFAULT_PATH)

# find_package(absl REQUIRED)
set(protobuf_MODULE_COMPATIBLE TRUE)
find_package(Protobuf CONFIG REQUIRED)
set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)


find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(trace_publisher src/trace_publisher_otlp.cpp)
ament_target_dependencies(trace_publisher rclcpp std_msgs)

# Link all required libraries
target_link_libraries(trace_publisher
  ${OPENTELEMETRY_CPP_LIBRARIES}
  protobuf
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  trace_publisher
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()