# Bai Thuc Hanh 04 - TurtleBot 4 Simulation, SLAM va Nav2

Du an thuc hien mo phong robot **TurtleBot 4** trong moi truong **Ignition Gazebo**, xay dung ban do 2D bang **SLAM Toolbox**, dinh vi robot bang **AMCL** va dieu huong tu dong khong va cham bang **Nav2** tren **ROS 2 Humble**. Mac dinh repository dung world **depot** vi nhe hon **maze**.


---

## Cau Truc Thu Muc

```text
tb4_maps/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── tb4_maps
├── maps/
│   ├── map.yaml              # Cau hinh OccupancyGrid: image, resolution, origin, thresholds
│   └── map.pgm               # Anh raster 2D cua ban do da tao bang SLAM
├── launch/
│   ├── tb4_simulation.launch.py
│   ├── tb4_slam.launch.py
│   └── tb4_nav2.launch.py
├── scripts/
│   ├── run_mapping.md
│   └── run_navigation.md
├── report/
│   ├── Bao_cao_Thuc_hanh_04.md
│   └── Bao_cao_Thuc_hanh_04.pdf
└── readme.md
```


---

## Yeu Cau He Thong

- **He dieu hanh:** Ubuntu 22.04 LTS native hoac WSL2 + Ubuntu 22.04 co GUI support.
- **ROS:** ROS 2 Humble Desktop.
- **Simulator:** TurtleBot 4 Simulator voi Ignition Gazebo.
- **Cac goi can dung:** TurtleBot 4 simulator, TurtleBot 4 navigation, SLAM Toolbox, Nav2 va teleop.

### Cai Dat Phu Thuoc

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-turtlebot4-simulator \
  ros-humble-turtlebot4-desktop \
  ros-humble-turtlebot4-navigation \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard
```

Nap moi truong ROS trong moi terminal:

```bash
source /opt/ros/humble/setup.bash
```

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Build Package Trong Repository

Dat repository trong workspace ROS 2, vi du:

```bash
mkdir -p ~/tb4_ws/src
cp -r /home/aionia/tb4_maps ~/tb4_ws/src/tb4_maps
cd ~/tb4_ws
colcon build --packages-select tb4_maps
source install/setup.bash
```
## Phan 1 - Chay TurtleBot 4 Simulation

Mo terminal 1 va khoi chay TurtleBot 4 trong world depot kem RViz2:

```bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=depot rviz:=true gazebo:=ignition
```

Ket qua can quan sat:

- Gazebo hien thi robot TurtleBot 4 trong moi truong depot.
- RViz2 hien thi robot model, du lieu LiDAR va cac frame TF.
- Cac topic co ban `/scan`, `/odom`, `/cmd_vel` ton tai va hoat dong.

### Kiem Tra Topic Co Ban

Mo terminal 2:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

Kiem tra du lieu LiDAR:

```bash
ros2 topic echo /scan --once
ros2 topic hz /scan
```

Kiem tra odometry:

```bash
ros2 topic echo /odom --once
ros2 topic hz /odom
```

Kiem tra robot nhan lenh van toc qua `/cmd_vel`:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Dung robot:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

```bash
ros2 launch tb4_maps tb4_simulation.launch.py
```
---

## Phan 2 - Xay Dung Ban Do Bang SLAM Toolbox

Giu Gazebo dang chay. Mo terminal 3 va khoi chay SLAM Toolbox:

```bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot4_navigation slam.launch.py
```

Hoac chay simulation kem SLAM bang launch file cua repository:

```bash
ros2 launch tb4_maps tb4_slam.launch.py
```

Mac dinh lenh tren dung `depot`. Neu muon quay lai maze:

```bash
ros2 launch tb4_maps tb4_slam.launch.py sim_world:=maze
```

Mo terminal 4 de dieu khien robot bang ban phim:

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Trong qua trinh mapping:

- Dieu khien robot di cham, uu tien toc do thang khoang `0.15 - 0.2 m/s`.
- Quay dau va re cua tu tu de giam loi scan matching.
- Di qua phan lon cac hanh lang va khu vuc robot co the di chuyen.
- Theo doi map tren RViz2, dam bao tuong va vat can khong bi nhan doi hoac lech ro ret.

### Luu Ban Do

Sau khi ban do on dinh, luu map bang `map_saver_cli`:

```bash
mkdir -p ~/tb4_ws/maps
cd ~/tb4_ws/maps
ros2 run nav2_map_server map_saver_cli -f map
```

Lenh tren tao ra:

```text
map.yaml
map.pgm
```

Copy map vao repository:

```bash
mkdir -p /home/aionia/tb4_maps/maps
cp ~/tb4_ws/maps/map.yaml /home/aionia/tb4_maps/maps/map.yaml
cp ~/tb4_ws/maps/map.pgm /home/aionia/tb4_maps/maps/map.pgm
```

Kiem tra file map:

```bash
ls -lh /home/aionia/tb4_maps/maps
cat /home/aionia/tb4_maps/maps/map.yaml
```

Neu muon xem rieng checklist mapping, doc [scripts/run_mapping.md](./scripts/run_mapping.md).

---

## Phan 3 - Localization Bang AMCL

Sau khi mapping xong:

1. Dung SLAM Toolbox va teleop bang `Ctrl + C`.
2. Khoi dong lai robot trong cung world da dung khi mapping neu can thiet.
3. Load ban do da tao va chay Nav2 localization.

Khoi chay Nav2 voi map trong repository:

```bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
  world:=depot rviz:=true gazebo:=ignition localization:=true nav2:=true \
  map:=/home/aionia/tb4_maps/maps/map.yaml
```

Neu da build package `tb4_maps`, co the dung:

```bash
ros2 launch tb4_maps tb4_nav2.launch.py map:=/home/aionia/tb4_maps/maps/map.yaml
```

Neu map duoc tao bang `maze` hoac `warehouse`, phai chay lai dung world do:

```bash
ros2 launch tb4_maps tb4_nav2.launch.py sim_world:=maze map:=/home/aionia/tb4_maps/maps/map.yaml
```

Thiet lap vi tri ban dau tren RViz2:

1. Chon cong cu **2D Pose Estimate**.
2. Click va keo mui ten tai vi tri robot dang dung trong Gazebo.
3. Dieu chinh huong mui ten trung voi huong robot thuc te.
4. Kiem tra cac tia laser `/scan` trung voi tuong tren ban do.

Neu laser scan khop voi map va robot pose on dinh, localization bang AMCL da thanh cong.


## Phan 3 - Navigation Tu Dong Voi Nav2

Khi localization da dung:

1. Tren RViz2, chon cong cu **Nav2 Goal**.
2. Dat goal thu nhat tai mot khu vuc trong world da map.
3. Quan sat global path, local costmap va robot di chuyen.
4. Lap lai voi it nhat 03 goal o cac khu vuc khac nhau.
5. Sau khi dat goal, khong dieu khien robot thu cong bang teleop.

Ket qua dat yeu cau khi robot:

- Tu lap ke hoach duong di tu vi tri hien tai den goal.
- Di theo duong da lap ke hoach.
- Tranh tuong va vat can trong world.
- Den duoc it nhat 03 goal khac nhau khong va cham.

Huong dan ngan gon cho buoc localization/Nav2 nam trong [scripts/run_navigation.md](./scripts/run_navigation.md).

---

## Cac Node, Package Va Topic Chinh

| Thanh phan | Package / Node chinh | Vai tro |
| :--- | :--- | :--- |
| Simulation | `turtlebot4_ignition_bringup` | Khoi chay TurtleBot 4 trong Ignition Gazebo va bridge du lieu sang ROS 2 |
| Sensor LiDAR | Simulation plugin / LiDAR driver | Publish du lieu quet laser len topic `/scan` |
| Odometry | `irobot_create_control` / simulation control | Publish uoc luong chuyen dong robot len `/odom` |
| Velocity command | `/cmd_vel` | Nhan lenh van toc tu teleop hoac Nav2 controller |
| Mapping | `slam_toolbox` | Tao OccupancyGrid `/map` tu `/scan`, `/odom` va TF |
| Map server | `nav2_map_server` | Load file `map.yaml` va publish static map |
| Localization | `nav2_amcl` | Uoc luong pose robot tren map bang Adaptive Monte Carlo Localization |
| Planner | `nav2_navfn_planner` | Lap duong di toan cuc tu start den goal |
| Costmap | `nav2_costmap_2d` | Tao global/local costmap de bieu dien vat can va vung an toan |
| Controller | `nav2_dwb_controller` | Sinh lenh `/cmd_vel` de robot bam duong va tranh va cham |
| Behavior Tree | `nav2_bt_navigator` | Quan ly quy trinh navigation tu khi nhan goal den khi hoan thanh |

### Cac Topic Quan Trong

| Topic | Kieu du lieu | Y nghia |
| :--- | :--- | :--- |
| `/scan` | `sensor_msgs/msg/LaserScan` | Du lieu LiDAR 2D dung cho SLAM, AMCL va costmap |
| `/odom` | `nav_msgs/msg/Odometry` | Odometry cua robot |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Lenh van toc dieu khien robot |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Ban do dang luoi occupancy |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Cay bien doi toa do giua `map`, `odom`, `base_link`, sensor |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Pose robot do AMCL uoc luong |
| `/plan` | `nav_msgs/msg/Path` | Duong di toan cuc do planner tao ra |

---

## Giai Thich Pipeline

### Mapping

Trong giai doan mapping, TurtleBot 4 publish du lieu LiDAR tren `/scan` va odometry tren `/odom`. SLAM Toolbox su dung hai nguon du lieu nay cung voi TF de uoc luong quy dao robot, dong thoi ghep cac lan quet laser thanh ban do occupancy grid. Khi robot di qua nhieu khu vuc va quay lai cac diem da thay, SLAM Toolbox co the thuc hien loop closure de giam sai so tich luy.

### Localization

Sau khi da co `map.yaml` va `map.pgm`, Nav2 load ban do bang `map_server`. AMCL nhan `/scan`, map tinh va pose ban dau tu RViz2 de uoc luong vi tri robot. Khi cac tia LiDAR trung voi vat can tren ban do, pose cua robot tren RViz2 phan anh dung vi tri thuc te trong Gazebo.

### Navigation

Khi nguoi dung dat Nav2 Goal, Nav2 planner tao global path tren ban do. Costmap bieu dien tuong, vat can va vung an toan xung quanh robot. Controller DWB tinh lenh van toc phu hop va publish len `/cmd_vel`, giup TurtleBot 4 bam theo duong di, dieu chinh cuc bo va tranh va cham cho den khi den goal.

---



## Tai Lieu Tham Khao

- TurtleBot 4 Simulator: <https://github.com/turtlebot/turtlebot4_simulator>
- TurtleBot 4 User Manual: <https://turtlebot.github.io/turtlebot4-user-manual/>
- ROS 2 Humble Documentation: <https://docs.ros.org/en/humble/>
- Nav2 Documentation: <https://docs.nav2.org/>
- SLAM Toolbox: <https://github.com/SteveMacenski/slam_toolbox>
