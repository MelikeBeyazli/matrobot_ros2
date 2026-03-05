# 08 – ROS ↔ Gazebo İletişimi (GZ Bridge)

Bu derste **ROS 2 ile Gazebo simülasyonu arasında veri alışverişinin nasıl yapıldığını** öğreneceğiz.

Gazebo simülasyonu kendi **mesaj sistemini** kullanır.
ROS ise **farklı mesaj tiplerine** sahiptir.

Bu nedenle **ROS ile Gazebo arasında doğrudan iletişim kurulamaz.**

Bu problemi çözmek için **ROS–Gazebo Bridge** kullanılır.

```
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

Bridge sistemi sayesinde **ROS ve Gazebo mesajları birbirine çevrilir.**

---

# GZ Bridge Nedir?

`ros_gz_bridge` paketi, ROS2 ile Gazebo arasında **mesaj çevirisi yapan bir köprüdür.**

Örneğin:

* ROS tarafında `/cmd_vel` mesajı yayınlanır.
* Bridge bu mesajı Gazebo mesaj tipine çevirir.
* Gazebo robotu hareket ettirir.

Aynı şekilde **sensör verileri de Gazebo’dan ROS’a aktarılabilir.**

Örnek veri akışı:

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
ROS Topic: /scan
```

---

# Matrobot Simülasyonunda Kullandığımız Topicler

Simülasyonda aşağıdaki veri akışlarını kullanıyoruz.

## ROS → Gazebo

Robot kontrol komutları:

```
/cmd_vel
```

## Gazebo → ROS

Sensör ve durum verileri:

```
odometry
tf
joint_states
lidar scan
imu
clock
```

---

# Bridge Konfigürasyonu

Bridge ayarları bir **YAML dosyasında** tanımlanır.

Genellikle şu konumda bulunur:

```
matrobot_simulation/config/gz_bridge.yaml
```

Bu dosya **ROS topicleri ile Gazebo topiclerini eşleştirir.**

---

# ROS → Gazebo (Robot Hareketi)

Robotu hareket ettirmek için `/cmd_vel` mesajı Gazebo’ya gönderilir.

```yaml
# cmd_vel: ROS -> Gazebo
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
```

Bu yapı sayesinde:

```
ROS /cmd_vel  →  Gazebo robot hareketi
```

---

# Gazebo → ROS (Odometry)

Gazebo robotun **odometry bilgisini** üretir ve ROS tarafına gönderir.

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

Robotun konum bilgisi Gazebo’dan **TF sistemi üzerinden** ROS’a aktarılır.

```yaml
# tf: Gazebo -> ROS
- ros_topic_name: "/tf"
  gz_topic_name: "/pose"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS
  lazy: true
```

Bu sayede robotun **TF ağacı ROS içinde görüntülenebilir.**

---

# Gazebo → ROS (Joint States)

Robotun eklem pozisyonları ROS tarafına aktarılır.

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

Gazebo’daki LiDAR sensörü ROS’a **LaserScan** mesajı gönderir.

```yaml
# lidar
- ros_topic_name: "/scan"
  gz_topic_name: "/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
  lazy: true
```

Bu sayede ROS içinde şu topic oluşur:

```
/scan
```

---

# Gazebo → ROS (IMU)

IMU sensör verileri Gazebo’dan ROS’a aktarılır.

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

# `lazy` Parametresi Ne İşe Yarar?

Bridge konfigürasyonunda sık görülen bir parametre:

```
lazy: true
```

Bu parametre şu anlama gelir:

> Eğer ROS tarafında bu topic’i dinleyen bir node yoksa bridge veri göndermez.

Bu sayede:

* Gereksiz mesaj üretimi engellenir
* Sistem performansı artar
* CPU ve ağ kullanımı azalır

---

# Resmi Dokümantasyon

`ros_gz_bridge` paketi ve desteklenen mesaj tipleri hakkında daha fazla bilgi için resmi ROS dokümantasyonunu inceleyebilirsiniz:

```md
https://docs.ros.org/en/jazzy/p/ros_gz_bridge/
```

Özellikle dokümantasyondaki şu bölümde **ROS ve Gazebo arasında çevrilebilen mesaj tiplerinin tam listesi** bulunmaktadır:

```
The following message types can be bridged for topics
```

Bu bölümde:

* ROS mesaj tipleri
* Gazebo mesaj tipleri
* desteklenen topic eşleşmeleri

detaylı olarak açıklanmaktadır.

---

# Bu Derste Ne Öğrendik?

Bu derste:

* ROS ve Gazebo’nun **farklı mesaj sistemleri kullandığını**
* `ros_gz_bridge` paketinin bu iki sistemi **birbirine bağladığını**
* Topic eşleştirmelerinin **YAML dosyası ile tanımlandığını**
* Robot kontrolü ve sensör verilerinin **bridge üzerinden aktarıldığını** öğrendik.

---
