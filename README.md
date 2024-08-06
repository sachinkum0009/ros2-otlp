# OTLP ROS2

## Build OTLP CPP

```bash
git clone https://github.com/open-telemetry/opentelemetry-cpp.git

cd opentelemetry-cpp

mkdir build

cd build

cmake -DBUILD_TESTING=OFF -DWITH_OTLP_HTTP=ON -DWITH_OTLP_GRPC=ON -DWITH_OTLP_=ON -DCMAKE_BUILD_TYPE=Release  ..

cmake --install . --prefix ../../otel-cpp
```

## Commands

```bash
colcon build --packages-select my_trace_pkg_cpp

source install/setup.bash

export LD_LIBRARY_PATH=/media/asus/backup/zzzzz/ros2/acceleration_robotics/trace_ws/dependencies/opentelemetry-cpp/build:$LD_LIBRARY_PATH

ros2 launch my_trace_pkg_cpp trace_publisher.launch.py
```

## Start Jaeger

```bash
cd my_trace_pkg_py/jaeger/

docker compose up -d

docker compose down
```