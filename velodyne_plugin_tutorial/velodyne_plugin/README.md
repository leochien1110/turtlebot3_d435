# Build
```bash
mkdir build
cd build
cmake -DCMAKE_CXX_STANDARD=17 ..
make
```

# Run gazebo and control spin velocity
Launch the ros master
```bash
roscore
```
Setup `libvelodyne_plugin.so` path and run Gazebo world
```bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$PWD/build
gazebo --verbose velodyne.world
```

Control the velocity
```bash
rostopic pub /my_velodyne/vel_cmd std_msgs/Float32 1.0
```