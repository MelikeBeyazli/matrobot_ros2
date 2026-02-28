## 1) ROS ortamını hazırla

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

> `echo $ROS_DISTRO` boşsa, ROS kurulumu / source işlemi yapılmamıştır.

---

## 2) Gerekli paketleri yükle (apt ile)

```bash
sudo apt update
```

> URDF / Xacro + görselleştirme

```bash
sudo apt install -y \
  ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-rviz2 \
  ros-$ROS_DISTRO-urdf-tutorial
```

> Navigasyon (Nav2) + mesaj paketleri

```bash
sudo apt install -y \
  ros-$ROS_DISTRO-nav2-bringup \
  ros-$ROS_DISTRO-navigation2 \
  ros-$ROS_DISTRO-nav2-common
```

> SLAM

```bash

sudo apt install -y \
  ros-$ROS_DISTRO-slam-toolbox
```

> Localization (EKF/UKF)

```bash
sudo apt install -y \
  ros-$ROS_DISTRO-robot-localization
```

> Repo içinde kullanıldığı görünen ek temel paketler

```bash
sudo apt install -y \
  ros-$ROS_DISTRO-tf2-ros
```

---

## 3) Gazebo + ROS köprüsü

```bash
sudo apt install -y
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-ros-gz-bridge
```
---

## 4) Workspace oluştur ve repoyu indir

```bash
mkdir -p ~/matro_ws/src
cd ~/matro_ws/src
git clone https://github.com/MelikeBeyazli/matrobot_ros2.git
```

---

## 5) Bağımlılıkları “unutmamak” için rosdep ile kur

Bu adım “ben bir şeyi unuttum mu?” problemini büyük ölçüde çözer:

```bash
cd ~/matro_ws

sudo apt install -y python3-rosdep
sudo rosdep init || true
rosdep update

rosdep install --from-paths src --ignore-src -r -y
```

---

## 6) Derle ve source et

```bash
cd ~/matro_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---
