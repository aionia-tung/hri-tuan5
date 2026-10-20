# Bai Thuc Hanh 05 - TurtleBot 4 Localization, Autonomous Navigation & Nhan Dien Vat The Voi YOLO

Su dung lai ban do `maps/map.yaml` + `maps/map.pgm` da quet o bai 04 (world **maze** cua TurtleBot 4) de:

1. Dinh vi TurtleBot 4 bang **AMCL** trong Ignition Gazebo.
2. Viet node **`goal_navigator`** (Python, `nav2_simple_commander`) tu dong gui robot qua 4 goal bang action `/navigate_to_pose`.
3. Viet node **`yolo_detector`** nhan anh camera OAK-D `/oakd/rgb/preview/image_raw`, chuyen sang OpenCV bang `cv_bridge`, nhan dien bang **YOLOv8n** (ultralytics), ve bounding box va in ket qua ra terminal.

> Huong dan bai 04 (SLAM, luu map) nam o [readme_tuan04.md](./readme_tuan04.md).
> Luu y: ban do trong `maps/` duoc quet trong world **maze** (20 x 20 m), khong phai depot.

---

## Cau Truc Thu Muc

```text
tb4_maps/
├── package.xml / setup.py / setup.cfg
├── tb4_maps/
│   ├── goal_navigator.py        # Node dieu huong qua cac goal (Phan 3)
│   └── yolo_detector.py         # Node nhan dien vat the YOLO (Phan 4)
├── launch/
│   ├── tb4_week5_sim.launch.py  # Gazebo + robot + map_server/AMCL + Nav2 + RViz2 (Phan 1)
│   ├── tb4_week5_apps.launch.py # yolo_detector + goal_navigator (Phan 3, 4)
│   └── tb4_*.launch.py          # launch cua bai 04
├── config/
│   ├── goals.yaml               # Toa do initial pose va cac goal (Phan 2)
│   ├── yolo.yaml                # Tham so YOLO (topic, model, confidence...)
│   └── tb4_week5.rviz           # RViz2: map, laser, costmap, plan, marker goal, anh YOLO
├── worlds/
│   └── maze_objects.sdf         # World maze + nguoi, ghe, ban de YOLO nhan dien
└── maps/
    ├── map.yaml
    └── map.pgm
```

---

## Cai Dat

Yeu cau: Ubuntu 22.04, ROS 2 Humble, TurtleBot 4 Simulator (Ignition Gazebo Fortress).

```bash
sudo apt update
sudo apt install -y ros-humble-turtlebot4-simulator ros-humble-turtlebot4-navigation \
  ros-humble-nav2-simple-commander ros-humble-cv-bridge

# CPU-only torch (nhe hon nhieu so voi ban CUDA mac dinh)
pip install --user torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install --user ultralytics "numpy<2" "opencv-python<4.12"
```

**Luu y tuong thich (quan trong):**

- `numpy<2`: `cv_bridge` cua ROS Humble duoc build voi NumPy 1.x, NumPy 2 se lam no loi khi import.
  `opencv-python>=4.12` bat buoc NumPy 2 nen phai ghim `<4.12`.
- torch keo theo `setuptools>=77` vao `~/.local`, ban nay lam `colcon build` package Python bi loi
  `canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'`. Neu gap, go no di
  de dung lai setuptools cua he thong:

  ```bash
  pip uninstall -y setuptools   # chi go ban trong ~/.local
  ```

### Build

```bash
cd /home/aionia/tb4_maps
source /opt/ros/humble/setup.bash
colcon build --packages-select tb4_maps
source install/setup.bash
```

---

## Phan 1 - Simulation, Load Ban Do Va Localization (AMCL)

Terminal 1:

```bash
cd /home/aionia/tb4_maps && source install/setup.bash
ros2 launch tb4_maps tb4_week5_sim.launch.py
```

Launch nay:

- Mo Ignition Gazebo voi world `maze_objects` (= world `maze` da dung de quet map + 8 vat the: 4 nguoi, 3 ghe, 1 ban).
  Cac model lay tu Gazebo Fuel (da cache san trong `~/.ignition/fuel` khi chay world warehouse).
- Spawn TurtleBot 4 (standard, dang o dock) tai goc world `(0, 0, 0)`.
- Chay `map_server` (nap `maps/map.yaml`) + `amcl` (localization) va Nav2.
- Mo RViz2 voi config `config/tb4_week5.rviz`.

Tuy chon:

| Tham so | Mac dinh | Y nghia |
| :--- | :--- | :--- |
| `world` | `maze_objects` | `maze` = world goc khong co vat the |
| `map` | `maps/map.yaml` | Ban do da quet |
| `max_speed` | `0.3` | Van toc thang toi da cua Nav2 (m/s). TurtleBot 4 goc: 0.26; Create 3 gioi han ~0.306 |
| `max_turn` | `1.9` | Van toc quay toi da (rad/s). TurtleBot 4 goc: 1.0 |
| `gui` | `true` | `false` = khong mo cua so Gazebo, mo phong nhanh hon (RViz2 + YOLO van hien) |
| `rviz` | `true` | Mo RViz2 |
| `lidar_ogre2` | `true` | Sua loi LiDAR (xem duoi) |

Vi du chay nhanh nhat: `ros2 launch tb4_maps tb4_week5_sim.launch.py gui:=false`

**Loi LiDAR (Ogre1) tren GPU Intel/Mesa:** plugin Sensors cua Create 3 dung render engine `ogre`; voi GPU Intel UHD +
Mesa 23, moi tia `/scan` deu bang `range_min` (0.164 m) -> AMCL troi, costmap co vat can bao quanh robot, planner bao
"failed to create plan". Kiem tra: `ros2 topic echo /scan --once` thay toan `0.164`.
Launch tu tao overlay `~/.cache/tb4_maps/ogre2_overlay` (ban sao `irobot_create_description` voi
`<render_engine>ogre2</render_engine>`) va dat len dau `AMENT_PREFIX_PATH` (mac dinh `lidar_ogre2:=true`).
Cach sua tham khao tu <https://github.com/linhnghu/turtlebot4-nav2-yolo>.

Nen cam sac laptop khi chay: o che do pin (governor `powersave`) real time factor chi ~0.15.

**Initial pose:** robot spawn tai goc world, trung voi goc khi bat dau SLAM, nen pose ban dau trong frame `map` la
`(0, 0, 0)`. Launch sinh file tham so AMCL (`~/.cache/tb4_maps/localization.yaml`) voi `set_initial_pose: true`
va `initial_pose` = pose spawn, nen AMCL dinh vi ngay khi khoi dong. Node `goal_navigator` cung publish `/initialpose`
de du phong. Co the dat thu cong bang **2D Pose Estimate** tren RViz2. Kiem tra localization dung khi cac tia `/scan` (mau do) trung voi tuong tren map va particle cloud hoi tu.

```bash
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_link
```

## Phan 2 - Toa Do Muc Tieu

Lay toa do bang **Publish Point** tren RViz2 + `ros2 topic echo /clicked_point`, hoac `ros2 topic echo /amcl_pose`.
Cac goal duoc chon o 4 khu vuc khac nhau, cach tuong it nhat 1.5 m (khong nam trong vung inflation 0.45 m),
va da kiem tra lien thong voi diem xuat phat tren map:

| Goal | Khu vuc | x (m) | y (m) | yaw (do) | Vat the trong tam nhin |
| :--- | :--- | ---: | ---: | ---: | :--- |
| Goal 1 | Phong phia Dong | 6.0 | -2.0 | 0 | nguoi, ghe |
| Goal 2 | Goc Dong Nam | 4.4 | -7.4 | -90 | ban, ghe gap |
| Goal 3 | Khu Tay Nam | -6.1 | -4.1 | 90 | nguoi dang goi dien |
| Goal 4 | Phong Tay Bac | -7.6 | 7.6 | 45 | nguoi, ghe |

Sua trong `config/goals.yaml` (khong can build lai neu chay voi `goals:=<duong dan>`).

## Phan 3 + 4 - Navigation Tu Dong Va Nhan Dien YOLO

Terminal 2 (sau khi Terminal 1 da in `Managed nodes are active`):

```bash
cd /home/aionia/tb4_maps && source install/setup.bash
ros2 launch tb4_maps tb4_week5_apps.launch.py
```

Hoac chay tung node:

```bash
ros2 run tb4_maps yolo_detector --ros-args --params-file install/tb4_maps/share/tb4_maps/config/yolo.yaml -p use_sim_time:=true
ros2 run tb4_maps goal_navigator --ros-args -r __node:=goal_navigator \
  --params-file install/tb4_maps/share/tb4_maps/config/goals.yaml -p use_sim_time:=true
```

Ket qua:

- Robot tu undock, lan luot di qua Goal 1 -> 2 -> 3 -> 4, terminal in quang duong con lai/ETA, cuoi cung in tong ket.
- Cua so OpenCV **YOLO - TurtleBot 4** hien anh camera kem bounding box, ten class, do tu tin, FPS.
- Terminal in: `Phat hien 2 vat the: person(0.87) @[x1, y1, x2, y2], chair(0.61) @[...]`.
- RViz2: marker goal (vang = cho, xanh duong = dang di, xanh la = da den), global path, costmap, panel anh YOLO.

Lan dau chay, `yolov8n.pt` (~6 MB) duoc tai ve `~/.cache/tb4_maps/`.

Kiem tra:

```bash
ros2 action list                 # /navigate_to_pose, /undock, ...
ros2 topic hz /yolo/image_annotated
ros2 topic echo /yolo/detections
```

---

## Node, Topic Va Action Chinh

| Thanh phan | Node | Vao | Ra |
| :--- | :--- | :--- | :--- |
| Map server | `map_server` | `maps/map.yaml` | `/map` |
| Localization | `amcl` | `/scan`, `/map`, `/initialpose`, TF `odom->base_link` | `/amcl_pose`, `/particle_cloud`, TF `map->odom` |
| Navigation | `bt_navigator` | action `/navigate_to_pose` | goi planner/controller/behavior |
| Global planner | `planner_server` (NavFn) | global costmap | `/plan` |
| Local controller | `controller_server` (DWB) | `/plan`, local costmap | `/cmd_vel` |
| Dieu huong (tu viet) | `goal_navigator` | `config/goals.yaml` | `/initialpose`, action `/undock`, `/navigate_to_pose`, `/goal_markers` |
| Nhan dien (tu viet) | `yolo_detector` | `/oakd/rgb/preview/image_raw` | `/yolo/image_annotated`, `/yolo/detections`, cua so OpenCV |

## Tai Lieu Tham Khao

- TurtleBot 4 Navigator: <https://turtlebot.github.io/turtlebot4-user-manual/tutorials/turtlebot4_navigator.html>
- Nav2 Simple Commander API: <https://docs.nav2.org/commander_api/index.html>
- Ultralytics YOLO: <https://docs.ultralytics.com/>
