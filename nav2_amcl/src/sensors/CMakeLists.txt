set(opentelemetry-cpp_DIR /home/asus/backup/zzzzz/ros2/acceleration_robotics/trace_ws/dependencies/otel-cpp/lib/cmake/opentelemetry-cpp)

# Use find_package to include OpenTelemetry C++
find_package(opentelemetry-cpp CONFIG REQUIRED NO_DEFAULT_PATH)

# find_package(absl REQUIRED)
set(protobuf_MODULE_COMPATIBLE TRUE)
find_package(Protobuf CONFIG REQUIRED)
set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)


find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(sensors_lib SHARED
  laser/laser.cpp
  laser/beam_model.cpp
  laser/likelihood_field_model.cpp
  laser/likelihood_field_model_prob.cpp
)
# map_update_cspace
target_link_libraries(sensors_lib pf_lib map_lib ${OPENTELEMETRY_CPP_LIBRARIES}
  protobuf)

# target_link_libraries(${library_name}
  
# )

install(TARGETS
  sensors_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
