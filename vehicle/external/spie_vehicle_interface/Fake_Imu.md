如何模擬遙控器控制車輛

1. 將檔案 `/home/spie/autoware/src/universe/autoware_universe/launch/tier4_simulator_launch/launch/simulator.launch.xml` 第223行左右更改為

```yaml
<include file="$(find-pkg-share tier4_localization_launch)/launch/pose_twist_fusion_filter/pose_twist_fusion_filter.launch.xml">
    <arg name="pose_instability_detector_param_path" value="$(find-pkg-share autoware_launch)/config/localization/pose_instability_detector.param.yaml"/>
</include>
```

2. 將檔案 `/home/spie/autoware/src/launcher/autoware_launch/autoware_launch/launch/planning_simulator.launch.xml` 第92行左右替換為

```yaml
<!-- <let name="launch_dummy_vehicle" value="false" if="$(var scenario_simulation)"/>
<let name="launch_dummy_vehicle" value="true" unless="$(var scenario_simulation)"/> -->
<let name="launch_dummy_vehicle" value="false"/>

```

3. 使用spie_vehicle_interface_test_exec當作interface

4. 使用以下指令開啟Autoware

```shell
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/NTUT-MAP vehicle_model:=spie_vehicle sensor_model:=spie_sensor_kit vehicle_simulation:=false localization_sim_mode:=pose_twist_estimator
```

這些流程的作用主要為
1. 關閉模擬車輛(`<let name="launch_dummy_vehicle" value="false"/>`)
2. 開啟自訂節點(透過 `vehicle_simulation:=false`)
3. 開啟ekf，來進行定位(透過 `localization_sim_mode:=pose_twist_estimator`)
4. 在自訂節點中偽造IMU(ekf最基礎需要IMU)

---
另外在使用Fake_IMU interface時，會有一些錯誤，需要找一下原因。屏蔽方法
	編輯檔案:`/home/spie/autoware/src/launcher/autoware_launch/autoware_launch/config/system/diagnostics/localization.yaml`
	註釋掉關於`/autoware/localization/sensor_fusion_status`的部份
這是一我的情況所作的調整。

---
如果想檢查為何會產生錯誤無法啟動Auto(按鈕為灰色)，可以使用`rqt_diagnostic_graph_monitor`
安裝方法:
```shell
cd ~/ros2_ws/src
git clone https://github.com/autowarefoundation/autoware_tools.git
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select rqt_diagnostic_graph_monitor
```
執行
```shell
rqt -s rqt_diagnostic_graph_monitor
```


