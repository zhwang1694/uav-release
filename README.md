# EVA系列机型使用手册

 ## 单t265机型及使用说明

 ### 1、启动相机及连接px4

```
roslaunch vision_to_mavros t265_all_nodes.launch
```

正常的启动结果如下：

>... logging to /home/nvidia/.ros/log/2d5faa4e-f1b0-11ea-b4e2-3249988e6a84/roslaunch-tegra-ubuntu-4174.log  
Checking log directory for disk usage. This may take awhile.  
Press Ctrl-C to interrupt  
Done checking log file disk usage. Usage is <1GB.    
started roslaunch server http://tegra-ubuntu:39220/  

>SUMMARY  

>CLEAR PARAMETERS  
 >* /mavros/   
 
>PARAMETERS
 >* /camera/realsense2_camera/accel_fps: 62
 >* /camera/realsense2_camera/accel_frame_id: camera_accel_frame
 >* /camera/realsense2_camera/accel_optical_frame_id: camera_accel_opti...
 >* /camera/realsense2_camera/align_depth: False
 >* /camera/realsense2_camera/aligned_depth_to_color_frame_id: camera_aligned_de...
 >* /camera/realsense2_camera/aligned_depth_to_fisheye1_frame_id: camera_aligned_de...
 >* /camera/realsense2_camera/aligned_depth_to_fisheye2_frame_id: camera_aligned_de...
 >* /camera/realsense2_camera/aligned_depth_to_fisheye_frame_id: camera_aligned_de...
 >* /camera/realsense2_camera/aligned_depth_to_infra1_frame_id: camera_aligned_de...
 >* /camera/realsense2_camera/aligned_depth_to_infra2_frame_id: camera_aligned_de...
 >* /camera/realsense2_camera/allow_no_texture_points: False
 >* /camera/realsense2_camera/base_frame_id: camera_link
 >* /camera/realsense2_camera/calib_odom_file: 
 >* /camera/realsense2_camera/clip_distance: -1.0
 >* /camera/realsense2_camera/color_fps: 30
 >* /camera/realsense2_camera/color_frame_id: camera_color_frame
 >* /camera/realsense2_camera/color_height: 480
 >* /camera/realsense2_camera/color_optical_frame_id: camera_color_opti...
 >* /camera/realsense2_camera/color_width: 640
 >* /camera/realsense2_camera/depth_fps: 30
 >* /camera/realsense2_camera/depth_frame_id: camera_depth_frame
 >* /camera/realsense2_camera/depth_height: 480
 >* /camera/realsense2_camera/depth_optical_frame_id: camera_depth_opti...
 >* /camera/realsense2_camera/depth_width: 640
 >* /camera/realsense2_camera/device_type: t265
 >* /camera/realsense2_camera/enable_accel: True
 >* /camera/realsense2_camera/enable_color: True
 >* /camera/realsense2_camera/enable_depth: True
 >* /camera/realsense2_camera/enable_fisheye1: False
 >* /camera/realsense2_camera/enable_fisheye2: False
 >* /camera/realsense2_camera/enable_fisheye: False
 >* /camera/realsense2_camera/enable_gyro: True
 >* /camera/realsense2_camera/enable_infra1: False
 >* /camera/realsense2_camera/enable_infra2: False
 >* /camera/realsense2_camera/enable_infra: False
 >* /camera/realsense2_camera/enable_pointcloud: False
 >* /camera/realsense2_camera/enable_pose: True
 >* /camera/realsense2_camera/enable_sync: False
 >* /camera/realsense2_camera/filters: 
 >* /camera/realsense2_camera/fisheye1_frame_id: camera_fisheye1_f...
 >* /camera/realsense2_camera/fisheye1_optical_frame_id: camera_fisheye1_o...
 >* /camera/realsense2_camera/fisheye2_frame_id: camera_fisheye2_f...
 >* /camera/realsense2_camera/fisheye2_optical_frame_id: camera_fisheye2_o...
 >* /camera/realsense2_camera/fisheye_fps: 30
 >* /camera/realsense2_camera/fisheye_frame_id: camera_fisheye_frame
 >* /camera/realsense2_camera/fisheye_height: 800
 >* /camera/realsense2_camera/fisheye_optical_frame_id: camera_fisheye_op...
 >* /camera/realsense2_camera/fisheye_width: 848
 >* /camera/realsense2_camera/gyro_fps: 200
 >* /camera/realsense2_camera/gyro_frame_id: camera_gyro_frame
 >* /camera/realsense2_camera/gyro_optical_frame_id: camera_gyro_optic...
 >* /camera/realsense2_camera/imu_optical_frame_id: camera_imu_optica...
 >* /camera/realsense2_camera/infra1_frame_id: camera_infra1_frame
 >* /camera/realsense2_camera/infra1_optical_frame_id: camera_infra1_opt...
 >* /camera/realsense2_camera/infra2_frame_id: camera_infra2_frame
 >* /camera/realsense2_camera/infra2_optical_frame_id: camera_infra2_opt...
 >* /camera/realsense2_camera/infra_fps: 30
 >* /camera/realsense2_camera/infra_height: 480
 >* /camera/realsense2_camera/infra_width: 640
 >* /camera/realsense2_camera/initial_reset: False
 >* /camera/realsense2_camera/json_file_path: 
 >* /camera/realsense2_camera/linear_accel_cov: 0.01
 >* /camera/realsense2_camera/odom_frame_id: camera_odom_frame
 >* /camera/realsense2_camera/pointcloud_texture_index: 0
 >* /camera/realsense2_camera/pointcloud_texture_stream: RS2_STREAM_COLOR
 >* /camera/realsense2_camera/pose_frame_id: camera_pose_frame
 >* /camera/realsense2_camera/pose_optical_frame_id: camera_pose_optic...
 >* /camera/realsense2_camera/publish_odom_tf: True
 >* /camera/realsense2_camera/publish_tf: True
 >* /camera/realsense2_camera/rosbag_filename: 
 >* /camera/realsense2_camera/serial_no: 
 >* /camera/realsense2_camera/tf_publish_rate: 0.0
 >* /camera/realsense2_camera/topic_odom_in: camera/odom_in
 >* /camera/realsense2_camera/unite_imu_method: 
 >* /camera/realsense2_camera/usb_port_id: 
 >* /gamma_world: -1.5707963
 >* /mavros/cmd/use_comp_id_system_control: False
 >* /mavros/conn/heartbeat_mav_type: ONBOARD_CONTROLLER
 >* /mavros/conn/heartbeat_rate: 1.0
 >* /mavros/conn/system_time_rate: 1.0
 >* /mavros/conn/timeout: 10.0
 >* /mavros/conn/timesync_rate: 10.0
 >* /mavros/distance_sensor/rangefinder_pub/field_of_view: 0.0
 >* /mavros/distance_sensor/rangefinder_pub/frame_id: lidar
 >* /mavros/distance_sensor/rangefinder_pub/id: 0
 >* /mavros/distance_sensor/rangefinder_pub/send_tf: False
 >* /mavros/distance_sensor/rangefinder_pub/sensor_position/x: 0.0
 >* /mavros/distance_sensor/rangefinder_pub/sensor_position/y: 0.0
 >* /mavros/distance_sensor/rangefinder_pub/sensor_position/z: -0.1
 >* /mavros/distance_sensor/rangefinder_sub/id: 1
 >* /mavros/distance_sensor/rangefinder_sub/orientation: PITCH_270
 >* /mavros/distance_sensor/rangefinder_sub/subscriber: True
 >* /mavros/fake_gps/eph: 2.0
 >* /mavros/fake_gps/epv: 2.0
 >* /mavros/fake_gps/fix_type: 3
 >* /mavros/fake_gps/geo_origin/alt: 408.0
 >* /mavros/fake_gps/geo_origin/lat: 47.3667
 >* /mavros/fake_gps/geo_origin/lon: 8.55
 >* /mavros/fake_gps/gps_rate: 5.0
 >* /mavros/fake_gps/mocap_transform: True
 >* /mavros/fake_gps/satellites_visible: 5
 >* /mavros/fake_gps/tf/child_frame_id: fix
 >* /mavros/fake_gps/tf/frame_id: map
 >* /mavros/fake_gps/tf/listen: False
 >* /mavros/fake_gps/tf/rate_limit: 10.0
 >* /mavros/fake_gps/tf/send: False
 >* /mavros/fake_gps/use_mocap: True
 >* /mavros/fake_gps/use_vision: False
 >* /mavros/fcu_protocol: v2.0
 >* /mavros/fcu_url: /dev/ttyACM0:57600
 >* /mavros/gcs_url: 
 >* /mavros/global_position/child_frame_id: base_link
 >* /mavros/global_position/frame_id: map
 >* /mavros/global_position/gps_uere: 1.0
 >* /mavros/global_position/rot_covariance: 99999.0
 >* /mavros/global_position/tf/child_frame_id: base_link
 >* /mavros/global_position/tf/frame_id: map
 >* /mavros/global_position/tf/global_frame_id: earth
 >* /mavros/global_position/tf/send: False
 >* /mavros/global_position/use_relative_alt: True
 >* /mavros/image/frame_id: px4flow
 >* /mavros/imu/angular_velocity_stdev: 0.0003490659 // 0...
 >* /mavros/imu/frame_id: base_link
 >* /mavros/imu/linear_acceleration_stdev: 0.0003
 >* /mavros/imu/magnetic_stdev: 0.0
 >* /mavros/imu/orientation_stdev: 1.0
 >* /mavros/landing_target/camera/fov_x: 2.0071286398
 >* /mavros/landing_target/camera/fov_y: 2.0071286398
 >* /mavros/landing_target/image/height: 480
 >* /mavros/landing_target/image/width: 640
 >* /mavros/landing_target/land_target_type: VISION_FIDUCIAL
 >* /mavros/landing_target/listen_lt: False
 >* /mavros/landing_target/mav_frame: LOCAL_NED
 >* /mavros/landing_target/target_size/x: 0.3
 >* /mavros/landing_target/target_size/y: 0.3
 >* /mavros/landing_target/tf/child_frame_id: camera_center
 >* /mavros/landing_target/tf/frame_id: landing_target
 >* /mavros/landing_target/tf/listen: False
 >* /mavros/landing_target/tf/rate_limit: 10.0
 >* /mavros/landing_target/tf/send: True
 >* /mavros/local_position/frame_id: map
 >* /mavros/local_position/tf/child_frame_id: base_link
 >* /mavros/local_position/tf/frame_id: map
 >* /mavros/local_position/tf/send: False
 >* /mavros/local_position/tf/send_fcu: False
 >* /mavros/mission/pull_after_gcs: True
 >* /mavros/mission/use_mission_item_int: True
 >* /mavros/mocap/use_pose: True
 >* /mavros/mocap/use_tf: False
 >* /mavros/odometry/estimator_type: 3
 >* /mavros/odometry/frame_tf/desired_frame: ned
 >* /mavros/plugin_blacklist: ['actuator_contro...
 >* /mavros/plugin_whitelist: []
 >* /mavros/px4flow/frame_id: px4flow
 >* /mavros/px4flow/ranger_fov: 0.118682
 >* /mavros/px4flow/ranger_max_range: 5.0
 >* /mavros/px4flow/ranger_min_range: 0.3
 >* /mavros/safety_area/p1/x: 1.0
 >* /mavros/safety_area/p1/y: 1.0
 >* /mavros/safety_area/p1/z: 1.0
 >* /mavros/safety_area/p2/x: -1.0
 >* /mavros/safety_area/p2/y: -1.0
 >* /mavros/safety_area/p2/z: -1.0
 >* /mavros/setpoint_accel/send_force: False
 >* /mavros/setpoint_attitude/reverse_thrust: False
 >* /mavros/setpoint_attitude/tf/child_frame_id: target_attitude
 >* /mavros/setpoint_attitude/tf/frame_id: map
 >* /mavros/setpoint_attitude/tf/listen: False
 >* /mavros/setpoint_attitude/tf/rate_limit: 50.0
 >* /mavros/setpoint_attitude/use_quaternion: False
 >* /mavros/setpoint_position/mav_frame: LOCAL_NED
 >* /mavros/setpoint_position/tf/child_frame_id: target_position
 >* /mavros/setpoint_position/tf/frame_id: map
 >* /mavros/setpoint_position/tf/listen: False
 >* /mavros/setpoint_position/tf/rate_limit: 50.0
 >* /mavros/setpoint_raw/thrust_scaling: 1.0
 >* /mavros/setpoint_velocity/mav_frame: LOCAL_NED
 >* /mavros/startup_px4_usb_quirk: False
 >* /mavros/sys/disable_diag: False
 >* /mavros/sys/min_voltage: 10.0
 >* /mavros/target_component_id: 1
 >* /mavros/target_system_id: 1
 >* /mavros/tdr_radio/low_rssi: 40
 >* /mavros/time/time_ref_source: fcu
 >* /mavros/time/timesync_avg_alpha: 0.6
 >* /mavros/time/timesync_mode: MAVLINK
 >* /mavros/vibration/frame_id: base_link
 >* /mavros/vision_pose/tf/child_frame_id: vision_estimate
 >* /mavros/vision_pose/tf/frame_id: map
 >* /mavros/vision_pose/tf/listen: False
 >* /mavros/vision_pose/tf/rate_limit: 10.0
 >* /mavros/vision_speed/listen_twist: True
 >* /mavros/vision_speed/twist_cov: True
 >* /mavros/wheel_odometry/child_frame_id: base_link
 >* /mavros/wheel_odometry/count: 2
 >* /mavros/wheel_odometry/frame_id: map
 >* /mavros/wheel_odometry/send_raw: True
 >* /mavros/wheel_odometry/send_twist: False
 >* /mavros/wheel_odometry/tf/child_frame_id: base_link
 >* /mavros/wheel_odometry/tf/frame_id: map
 >* /mavros/wheel_odometry/tf/send: True
 >* /mavros/wheel_odometry/use_rpm: False
 >* /mavros/wheel_odometry/vel_error: 0.1
 >* /mavros/wheel_odometry/wheel0/radius: 0.05
 >* /mavros/wheel_odometry/wheel0/x: 0.0
 >* /mavros/wheel_odometry/wheel0/y: -0.15
 >* /mavros/wheel_odometry/wheel1/radius: 0.05
 >* /mavros/wheel_odometry/wheel1/x: 0.0
 >* /mavros/wheel_odometry/wheel1/y: 0.15
 >* /output_rate: 30
 >* /pitch_cam: 0
 >* /roll_cam: 0
 >* /rosdistro: kinetic
 >* /rosversion: 1.12.14
 >* /source_frame_id: /camera_link
 >* /target_frame_id: /camera_odom_frame
 >* /yaw_cam: 0

>NODES
  /camera/  
    realsense2_camera (nodelet/nodelet)  
    realsense2_camera_manager (nodelet/nodelet)  
  /  
    camera_to_mavros (vision_to_mavros/vision_to_mavros_node)  
    mavros (mavros/mavros_node)  

>auto-starting new master  
process[master]: started with pid [4187]  
ROS_MASTER_URI=http://localhost:11311  

>setting /run_id to 2d5faa4e-f1b0-11ea-b4e2-3249988e6a84  
process[rosout-1]: started with pid [4254]  
started core service [/rosout]   
process[camera/realsense2_camera_manager-2]: started with pid [4280]  
process[camera/realsense2_camera-3]: started with pid [4281]  
process[mavros-4]: started with pid [4282]  
process[camera_to_mavros-5]: started with pid [4293]  
[ INFO] [1599554960.174038640]: Initializing nodelet with 6 worker threads.  
[ INFO] [1599554960.278217776]: Get target_frame_id parameter: /camera_odom_frame  
[ INFO] [1599554960.291510192]: Get source_frame_id parameter: /camera_link  
[ INFO] [1599554960.296266864]: Get output_rate parameter: 30.000000  
[ INFO] [1599554960.298985744]: Get gamma_world parameter: -1.570796  
[ INFO] [1599554960.301745136]: Get roll_cam parameter: 0.000000  
[ INFO] [1599554960.305294032]: Get pitch_cam parameter: 0.000000  
[ INFO] [1599554960.314098672]: Get yaw_cam parameter: 0.000000  
[ INFO] [1599554960.527044880]: FCU URL: /dev/ttyACM0:57600  
[ INFO] [1599554960.536397872]: serial0: device: /dev/ttyACM0 @ 57600 bps  
[ INFO] [1599554960.537818800]: GCS bridge disabled  
[ INFO] [1599554960.622632720]: Plugin 3dr_radio loaded  
[ INFO] [1599554960.633180048]: Plugin 3dr_radio initialized  
[ INFO] [1599554960.633366448]: Plugin actuator_control blacklisted  
[ INFO] [1599554960.666424080]: Plugin adsb loaded  
[ INFO] [1599554960.692442320]: Plugin adsb initialized  
[ INFO] [1599554960.692626480]: Plugin altitude blacklisted  
[ INFO] [1599554960.693228880]: Plugin cam_imu_sync loaded  
[ INFO] [1599554960.699339728]: Plugin cam_imu_sync initialized  
[ INFO] [1599554960.700428752]: Plugin command loaded  
[ INFO] [1599554960.758768464]: Plugin command initialized  
[ INFO] [1599554960.759433360]: Plugin companion_process_status loaded  
[ INFO] [1599554960.782509872]: Plugin companion_process_status initialized  
[ INFO] [1599554960.782706480]: Plugin debug_value blacklisted  
[ INFO] [1599554960.783253456]: Plugin distance_sensor loaded  
[ INFO] [1599554960.866735728]: Plugin distance_sensor initialized  
[ INFO] [1599554960.867426128]: Plugin fake_gps loaded  
[ INFO] [1599554960.978992080]: Plugin fake_gps initialized  
[ INFO] [1599554960.979088944]: Plugin ftp blacklisted  
[ INFO] [1599554960.979535504]: Plugin global_position loaded  
[ INFO] [1599554961.122929584]: Plugin global_position initialized  
[ INFO] [1599554961.123550000]: Plugin gps_rtk loaded  
[ INFO] [1599554961.147142416]: Plugin gps_rtk initialized  
[ INFO] [1599554961.147322288]: Plugin hil blacklisted  
[ INFO] [1599554961.148034640]: Plugin home_position loaded  
[ INFO] [1599554961.182956272]: Plugin home_position initialized  
[ INFO] [1599554961.183788176]: Plugin imu loaded  
[ INFO] [1599554961.244168400]: Plugin imu initialized  
[ INFO] [1599554961.244804560]: Plugin landing_target loaded  
[ INFO] [1599554961.397873008]: Plugin landing_target initialized  
[ INFO] [1599554961.398609584]: Plugin local_position loaded  
[ INFO] [1599554961.460164880]: Plugin local_position initialized  
[ INFO] [1599554961.460783440]: Plugin log_transfer loaded  
[ INFO] [1599554961.486906320]: Plugin log_transfer initialized  
[ INFO] [1599554961.487715952]: Plugin manual_control loaded  
[ INFO] [1599554961.519067120]: Plugin manual_control initialized  
[ INFO] [1599554961.519578256]: Plugin mocap_pose_estimate loaded  
[ INFO] [1599554961.541344752]: Plugin mocap_pose_estimate initialized  
[ INFO] [1599554961.544259056]: Plugin mount_control loaded  
[ INFO] [1599554961.573551152]: Plugin mount_control initialized  
[ INFO] [1599554961.574087664]: Plugin obstacle_distance loaded  
[ INFO] [1599554961.600458736]: Plugin obstacle_distance initialized  
[ INFO] [1599554961.601089520]: Plugin odom loaded   
[ INFO] [1599554961.639177360]: Plugin odom initialized  
[ INFO] [1599554961.640106320]: Plugin onboard_computer_status loaded  
[ INFO] [1599554961.663778960]: Plugin onboard_computer_status initialized  
[ INFO] [1599554961.664567600]: Plugin param loaded  
[ INFO] [1599554961.691886512]: Plugin param initialized  
[ INFO] [1599554961.692076656]: Plugin px4flow blacklisted  
[ INFO] [1599554961.692662160]: Plugin rangefinder loaded  
[ INFO] [1599554961.697919472]: Plugin rangefinder initialized  
[ INFO] [1599554961.698652016]: Plugin rc_io loaded  
[ INFO] [1599554961.731852496]: Plugin rc_io initialized  
[ INFO] [1599554961.732053584]: Plugin safety_area blacklisted  
[ INFO] [1599554961.732705648]: Plugin setpoint_accel loaded  
[ INFO] [1599554961.766805904]: Plugin setpoint_accel initialized  
[ INFO] [1599554961.767545008]: Plugin setpoint_attitude loaded  
[ INFO] [1599554961.858055344]: Plugin setpoint_attitude initialized  
[ INFO] [1599554961.858781328]: Plugin setpoint_position loaded  
[ INFO] [1599554962.000669072]: Plugin setpoint_position initialized  
[ INFO] [1599554962.001729328]: Plugin setpoint_raw loaded  
[ INFO] [1599554962.097878224]: Plugin setpoint_raw initialized  
[ INFO] [1599554962.098622832]: Plugin setpoint_trajectory loaded  
[ INFO] [1599554962.146045136]: Plugin setpoint_trajectory initialized  
[ INFO] [1599554962.146840784]: Plugin setpoint_velocity loaded  
[ INFO] [1599554962.207052560]: Plugin setpoint_velocity initialized  
[ INFO] [1599554962.208248912]: Plugin sys_status loaded  
[ INFO] [1599554962.312260848]: Plugin sys_status initialized  
[ INFO] [1599554962.313012656]: Plugin sys_time loaded  
[ INFO] [1599554962.378784944]: TM: Timesync mode: MAVLINK  
[ INFO] [1599554962.391551568]: Plugin sys_time initialized  
[ INFO] [1599554962.392347984]: Plugin trajectory loaded  
[ INFO] [1599554962.464534608]: Plugin trajectory initialized  
[ INFO] [1599554962.465825776]: Plugin vfr_hud loaded  
[ INFO] [1599554962.477453648]: Plugin vfr_hud initialized   
[ INFO] [1599554962.477934352]: Plugin vibration blacklisted  
[ INFO] [1599554962.479557776]: Plugin vision_pose_estimate loaded   
[ INFO] [1599554962.649547152]: Plugin vision_pose_estimate initialized   
[ INFO] [1599554962.649932240]: Plugin vision_speed_estimate blacklisted   
[ INFO] [1599554962.651368880]: Plugin waypoint loaded   
[ INFO] [1599554962.764549168]: Plugin waypoint initialized  
[ INFO] [1599554962.765502160]: Plugin wheel_odometry blacklisted  
[ INFO] [1599554962.766899344]: Plugin wind_estimation loaded  
[ INFO] [1599554962.777979248]: Plugin wind_estimation initialized  
[ INFO] [1599554962.778500496]: Built-in SIMD instructions: ARM NEON  
[ INFO] [1599554962.778817840]: Built-in MAVLink package version: 2020.7.7  
[ INFO] [1599554962.779169936]: Known MAVLink dialects: common ardupilotmega ASLUAV   autoquad icarous matrixpilot paparazzi slugs standard uAvionix ualberta  
[ INFO] [1599554962.779452560]: MAVROS started. MY ID 1.240, TARGET ID 1.1  
[ INFO] [1599554962.798148272]: IMU: Scaled IMU message used.  
[ INFO] [1599554962.800695312]: IMU: High resolution IMU detected!  
[ INFO] [1599554962.801976048]: IMU: Attitude quaternion IMU detected!  
[ INFO] [1599554962.820178896]: RealSense ROS v2.2.15  
[ INFO] [1599554962.820332688]: Built with LibRealSense v2.36.0  
[ INFO] [1599554962.820435408]: Running with LibRealSense v2.36.0  
[ INFO] [1599554962.944797712]:    
[ INFO] [1599554962.980719664]: Device with serial number 908412111246 was found.  
[ INFO] [1599554962.980929296]: Device with physical ID 2-1-2 was found.   
[ INFO] [1599554962.981051632]: Device with name Intel RealSense T265 was found.  
[ INFO] [1599554962.984570864]: Device with port number 2-1 was found.    
[ INFO] [1599554963.004930448]: No calib_odom_file. No input odometry accepted.  
[ INFO] [1599554963.005256624]: getParameters...  
[ INFO] [1599554963.084766576]: CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot  
[ INFO] [1599554963.096536432]: IMU: High resolution IMU detected!  
[ INFO] [1599554963.096803696]: IMU: Attitude quaternion IMU detected!  
[ WARN] [1599554963.334543760]: "camera_odom_frame" passed to lookupTransform argument   target_frame does not exist.   
[ INFO] [1599554963.583262096]: setupDevice...   
[ INFO] [1599554963.583420016]: JSON file is not provided  
[ INFO] [1599554963.583506416]: ROS Node Namespace: camera  
[ INFO] [1599554963.583582672]: Device Name: Intel RealSense T265  
[ INFO] [1599554963.583685168]: Device Serial No: 908412111246  
[ INFO] [1599554963.583759664]: Device physical port: 2-1-2  
[ INFO] [1599554963.583826416]: Device FW version: 0.2.0.951  
[ INFO] [1599554963.583888784]: Device Product ID: 0x0B37  
[ INFO] [1599554963.583959472]: Enable PointCloud: Off  
[ INFO] [1599554963.584016784]: Align Depth: Off  
[ INFO] [1599554963.584076944]: Sync Mode: Off  
[ INFO] [1599554963.584194960]: Device Sensors:   
[ INFO] [1599554963.584575568]: Tracking Module was found.  
[ INFO] [1599554963.584715920]: (Depth, 0) sensor isn't supported by current device! --   Skipping...  
[ INFO] [1599554963.584791504]: (Color, 0) sensor isn't supported by current device! -- Skipping...   
[ INFO] [1599554963.584908208]: num_filters: 0   
[ INFO] [1599554963.584983568]: Setting Dynamic reconfig parameters.    
[ INFO] [1599554963.675092848]: Done Setting Dynamic reconfig parameters.   
[ INFO] [1599554963.676108944]: setupPublishers...  
[ INFO] [1599554963.702274896]: setupStreams... 
[ INFO] [1599554963.702504176]: insert Gyro to Tracking Module  
[ INFO] [1599554963.702706128]: insert Accel to Tracking Module 
[ INFO] [1599554963.702825584]: insert Pose to Tracking Module  
[ INFO] [1599554963.742834480]: SELECTED BASE:Pose, 0       
[ INFO] [1599554963.775304240]: RealSense Node Is Up!   
[ INFO] [1599554964.094405424]: WP: Using MISSION_ITEM_INT  
[ INFO] [1599554964.094668848]: VER: 1.1: Capabilities         0x000000000000e4ef   
[ INFO] [1599554964.094839696]: VER: 1.1: Flight software:     010a01ff (e0f016c2b3000000)  
[ INFO] [1599554964.094975952]: VER: 1.1: Middleware software: 010a01ff (e0f016c2b3000000)  
[ INFO] [1599554964.095091120]: VER: 1.1: OS software:         071d00ff (427238133be2b0ec)  
[ INFO] [1599554964.095288624]: VER: 1.1: Board hardware:      00000032  
[ INFO] [1599554964.095435568]: VER: 1.1: VID/PID:             26ac:0032    
[ INFO] [1599554964.095572304]: VER: 1.1: UID:                 3138511935343633 


可能的错误列表：  
（1）Onboard Lost
>[ERROR] [1599642267.249704048]: FCU: Onboard controller lost  
 [ INFO] [1599642267.299322032]: FCU: Onboard controller regained

该错误会自动修正，无需处理

输入如下命令查看所有开启节点：

```
rostopic list
```

>/body_frame/path  
/camera/accel/imu_info  
/camera/accel/sample  
/camera/gyro/imu_info  
/camera/gyro/sample  
/camera/odom/sample  
/camera/realsense2_camera_manager/bond  
/camera/tracking_module/parameter_descriptions  
/camera/tracking_module/parameter_updates  
/diagnostics  
/mavlink/from  
/mavlink/to  
/mavros/adsb/send  
/mavros/adsb/vehicle  
/mavros/battery  
/mavros/cam_imu_sync/cam_imu_stamp  
/mavros/companion_process/status  
/mavros/distance_sensor/rangefinder_pub  
/mavros/distance_sensor/rangefinder_sub  
/mavros/estimator_status  
/mavros/extended_state  
/mavros/fake_gps/mocap/tf  
/mavros/global_position/compass_hdg  
/mavros/global_position/global  
/mavros/global_position/gp_lp_offset  
/mavros/global_position/gp_origin  
/mavros/global_position/home  
/mavros/global_position/local  
/mavros/global_position/raw/fix  
/mavros/global_position/raw/gps_vel  
/mavros/global_position/raw/satellites  
/mavros/global_position/rel_alt  
/mavros/global_position/set_gp_origin  
/mavros/gps_rtk/send_rtcm  
/mavros/home_position/home  
/mavros/home_position/set  
/mavros/imu/data  
/mavros/imu/data_raw  
/mavros/imu/diff_pressure  
/mavros/imu/mag  
/mavros/imu/static_pressure  
/mavros/imu/temperature_baro  
/mavros/imu/temperature_imu  
/mavros/landing_target/lt_marker  
/mavros/landing_target/pose  
/mavros/landing_target/pose_in  
/mavros/local_position/accel  
/mavros/local_position/odom  
/mavros/local_position/pose  
/mavros/local_position/pose_cov  
/mavros/local_position/velocity_body  
/mavros/local_position/velocity_body_cov  
/mavros/local_position/velocity_local  
/mavros/log_transfer/raw/log_data  
/mavros/log_transfer/raw/log_entry  
/mavros/manual_control/control  
/mavros/manual_control/send  
/mavros/mission/reached  
/mavros/mission/waypoints  
/mavros/mocap/pose  
/mavros/mount_control/command  
/mavros/mount_control/orientation  
/mavros/obstacle/send  
/mavros/odometry/in  
/mavros/odometry/out  
/mavros/onboard_computer/status  
/mavros/param/param_value  
/mavros/radio_status  
/mavros/rangefinder/rangefinder  
/mavros/rc/in  
/mavros/rc/out  
/mavros/rc/override  
/mavros/setpoint_accel/accel  
/mavros/setpoint_attitude/cmd_vel  
/mavros/setpoint_attitude/thrust  
/mavros/setpoint_position/global  
/mavros/setpoint_position/global_to_local  
/mavros/setpoint_position/local  
/mavros/setpoint_raw/attitude  
/mavros/setpoint_raw/global  
/mavros/setpoint_raw/local  
/mavros/setpoint_raw/target_attitude  
/mavros/setpoint_raw/target_global  
/mavros/setpoint_raw/target_local  
/mavros/setpoint_trajectory/desired  
/mavros/setpoint_trajectory/local  
/mavros/setpoint_velocity/cmd_vel  
/mavros/setpoint_velocity/cmd_vel_unstamped  
/mavros/state  
/mavros/statustext/recv  
/mavros/statustext/send  
/mavros/time_reference  
/mavros/timesync_status  
/mavros/trajectory/desired  
/mavros/trajectory/generated  
/mavros/trajectory/path  
/mavros/vfr_hud  
/mavros/vision_pose/pose  
/mavros/vision_pose/pose_cov  
/mavros/wind_estimation   
/rosout  
/rosout_agg  
/tf  
/tf_static  

主要通过以下两个节点输出检查设备运行情况：  
1、px4连接是否正常
```
rostopic echo /mavros/state
```

如果```connected```为```True```,则px4连接正常；如果为False，则重新插拔px4连接线，重新启动第一条命令。

>header:   
&nbsp;&nbsp;&nbsp;&nbsp;seq: 182  
&nbsp;&nbsp;&nbsp;&nbsp;stamp:   
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;secs: 1599555144  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;nsecs:  79931888  
&nbsp;&nbsp;&nbsp;&nbsp;frame_id: ''  
connected: True  
armed: False  
guided: False  
manual_input: True  
mode: "MANUAL"  
system_status: 3  

2、realsense t265是否连接正常
```
rostopic echo /mavros/local_position/pose
```
如下为飞机在局部坐标系下的位姿，移动飞机以检查飞机的x、y、z坐标是否正常

>header:   
&nbsp;&nbsp;&nbsp;&nbsp;  seq: 0  
&nbsp;&nbsp;&nbsp;&nbsp;  stamp:   
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;    secs: 1599555201  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;    nsecs: 522874752  
&nbsp;&nbsp;&nbsp;&nbsp;  frame_id: "map"  
pose:   
&nbsp;&nbsp;&nbsp;&nbsp;  position:   
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;    x: 0.00277996785007  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;    y: 0.00150535104331  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;    z: 0.00161666492932  
&nbsp;&nbsp;&nbsp;&nbsp;  orientation:   
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;    x: -0.000138912761128  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;    y: 0.00146321144005  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;    z: -0.707213626349  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;    w: -0.706998503031  

飞机的xyz坐标系朝向如下：


| z&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;   / y  
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;   /  
|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;  /  
|&nbsp;&nbsp;&nbsp; /  
|&nbsp;&nbsp;/  
O — — — x

