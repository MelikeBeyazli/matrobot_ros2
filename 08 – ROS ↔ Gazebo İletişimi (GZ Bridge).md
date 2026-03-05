
# 08 – ROS ↔ Gazebo İletişimi (GZ Bridge)

Bu derste ROS2 ile Gazebo simülasyonu arasında nasıl veri alışverişi yapıldığını öğreneceğiz.

Gazebo simülasyonu kendi mesaj sistemini kullanır.  
ROS ise kendi mesaj tiplerine sahiptir.

Bu nedenle ROS ile Gazebo arasında doğrudan iletişim kurulamaz.

Bu problemi çözmek için **ROS–Gazebo Bridge** kullanılır.


ROS2 Nodes
│
│
ROS Topics
│
│
ros_gz_bridge
│
│
Gazebo Topics
│
Gazebo Simulation

```

Bridge sistemi sayesinde ROS ve Gazebo mesajları birbirine çevrilir.

---

# GZ Bridge Nedir?

`ros_gz_bridge` paketi ROS2 ile Gazebo arasında **mesaj çevirisi yapan bir köprüdür.**

Örneğin:

- ROS tarafında `/cmd_vel` yayınlanır
- Bridge bu mesajı Gazebo mesajına çevirir
- Gazebo robotu hareket ettirir

Aynı şekilde sensör verileri de Gazebo’dan ROS’a aktarılır.

Örnek:

```

Gazebo LiDAR
│
│
LaserScan
│
│
ros_gz_bridge
│
│
ROS Topic /scan

```

---

# Matrobot Simülasyonunda Kullandığımız Topicler

Simülasyonda aşağıdaki veri akışlarını kullanıyoruz.

```

ROS → Gazebo
cmd_vel

```
```

Gazebo → ROS
odometry
tf
joint_states
lidar scan
imu
clock

```

---

# Bridge Konfigürasyonu

Bridge ayarlarını bir **YAML dosyasında** tanımlıyoruz.

Bu dosya genellikle:

```

matrobot_simulation/config/gz_bridge.yaml

````

içinde bulunur.

---

# ROS → Gazebo

Robotu hareket ettirmek için `/cmd_vel` topic’i Gazebo’ya gönderilir.

```yaml
# cmd_vel: ROS -> Gazebo
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
````

Bu yapı sayesinde:

```
ROS cmd_vel → Gazebo robot hareketi
```

---

# Gazebo → ROS (Odometry)

Gazebo robotun odometry bilgisini üretir ve ROS tarafına gönderir.

```yaml
# odom: Gazebo -> ROS
- ros_topic_name: "/odometry/wheel"
  gz_topic_name: "/odometry/wheel"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
  lazy: true
```

---

# Gazebo → ROS (TF)

Robotun konumu Gazebo'dan TF olarak ROS tarafına aktarılır.

```yaml
# tf: Gazebo -> ROS
- ros_topic_name: "/tf"
  gz_topic_name: "/pose"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS
  lazy: true
```

Bu sayede robotun **TF ağacı ROS içinde görünür.**

---

# Gazebo → ROS (Joint States)

Robot eklemlerinin pozisyonları ROS’a aktarılır.

```yaml
# joint states
- ros_topic_name: "/joint_states"
  gz_topic_name: "/joint_states"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS
  lazy: true
```

---

# Gazebo → ROS (LiDAR)

Gazebo’daki LiDAR sensörü ROS’a `LaserScan` mesajı gönderir.

```yaml
# lidar
- ros_topic_name: "/scan"
  gz_topic_name: "/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
  lazy: true
```

Bu sayede ROS içinde:

```
/scan
```

topic’i oluşur.

---

# Gazebo → ROS (IMU)

IMU sensör verileri de Gazebo’dan ROS’a aktarılır.

```yaml
# imu
- ros_topic_name: "/imu/data"
  gz_topic_name: "/imu/data"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS
  lazy: true
```

---

# Gazebo → ROS (Clock)

Simülasyon zamanını ROS’a aktarmak için `/clock` topic’i kullanılır.

```yaml
# clock
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
```

Bu sayede ROS sistemleri **simülasyon zamanını kullanabilir.**

---

# lazy Parametresi Ne İşe Yarar?

Bridge konfigürasyonunda sık görülen bir parametre:

```
lazy: true
```

Bu parametre şunu ifade eder:

> Eğer ROS tarafında bu topic’i dinleyen bir node yoksa bridge veri göndermez.

Bu sayede sistem gereksiz mesaj üretmez ve performans artar.

---

# Bu Derste Ne Öğrendik?

Bu derste:

* ROS ile Gazebo’nun farklı mesaj sistemleri kullandığını
* `ros_gz_bridge` paketinin bu iki sistemi bağladığını
* Topic eşleştirmelerinin YAML dosyası ile tanımlandığını
* Robot hareketi ve sensör verilerinin bu bridge üzerinden aktarıldığını öğrendik.
---
