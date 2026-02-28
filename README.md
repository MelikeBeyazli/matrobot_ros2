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

## 3) (Simülasyon kullanacaksan) Gazebo + ROS köprüsü

Dokümanlarınız Gazebo Harmonic + ROS 2 Jazzy kombinasyonundan bahsediyor. ([GitHub][6])
ROS tarafındaki en pratik kurulum “ros-gz” meta paketi:

```bash
sudo apt install -y
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-ros-gz-bridge
```

Bu paket ROS–Gazebo entegrasyonu için önerilen Gazebo kütüphanelerini kurar. ([gazebosim.org][7])
Ayrıca ROS+Gazebo Sim için `ros_gz_sim` paket dokümantasyonu da bu entegrasyonu anlatır. ([ROS][8])
Bridge (köprü) kullanımı `ros_gz_bridge` ile yapılır. ([gazebosim.org][9])

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
