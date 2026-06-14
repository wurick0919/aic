
Use https://playcanvas.com/model-viewer to view the mesh(.glb files in aic_asset)

### Testing with bag record

aic don't use regular dds, in new ternimal, start the zenoh router
```bash
pixi run ros2 run rmw_zenoh_cpp rmw_zenohd
```

in another terminal, replay the bag record
```bash
pixi run ros2 bag play my_recording/my_recording_0.mcap
```

Use the following command to build, instead of colcon build
```bash
pixi reinstall ros-kilted-my-policy-node
```

Run the simplified debug node. This will pop up window for recorded image from bag, and the depth image
```bash
pixi run ros2 run my_policy_node aic_model_debug
```
### Testing with gazebo env
Enter pixi
```bash
pixi shell
```

Enter distobox
```bash
export DBX_CONTAINER_MANAGER=docker
distrobox enter -r aic_eval
```
Inside Distrobox, manually update the xacro and config for gazebo, using container password
```bash
sudo cp "$HOME/ws_aic/src/aic/aic_assets/models/Basler Camera/basler_camera_macro.xacro" "/ws_aic/install/share/aic_assets/models/Basler Camera/basler_camera_macro.xacro"
sudo cp "$HOME/ws_aic/src/aic/aic_bringup/config/ros_gz_bridge_config.yaml" "/ws_aic/install/share/aic_bringup/config/ros_gz_bridge_config.yaml"
sudo cp "$HOME/ws_aic/src/aic/aic_engine/config/sample_config.yaml" "/ws_aic/install/share/aic_engine/config/sample_config.yaml"
```

Source the updates
```bash
. /ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true;transport/shared_memory/transport_optimization/pool_size=536870912'
```
Launch gazebo with zenoh router, enable aic_engine to spawn objects
```bash
/entrypoint.sh ground_truth:=true start_aic_engine:=true launch_rviz:=false
```

or, launch gazebo, when zenoh router is on
```bash
ros2 launch aic_bringup/launch/aic_gz_bringup.launch.py
```
in another terminal in pixi env
```bash
export PYTORCH_ALLOC_CONF=expandable_segments:True
ros2 run my_policy_node aic_model_depth --ros-args -p use_sim_time:=true -p policy:=my_policy_node.MyFoundationPosePolicy
```

