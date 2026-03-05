# 10 – Anlık Konum Tespiti-Haritalandırma-Navigasyon

# Robot Localization (EKF) – SLAM – Nav2

Mobil robotların otonom hareket edebilmesi için genellikle üç temel sistem birlikte çalışır:

1. **Robot Localization (EKF)**
2. **SLAM (Harita çıkarma)**
3. **Nav2 (Navigasyon sistemi)**

Bu üç sistem birlikte çalışarak robotun:

* nerede olduğunu bilmesini
* bulunduğu ortamın haritasını oluşturmasını
* hedef noktaya otonom olarak gitmesini

sağlar.

---

# Robot Localization (EKF)

Robot Localization paketindeki **EKF (Extended Kalman Filter)** node’u robotun konumunu daha doğru hesaplamak için farklı sensör verilerini birleştirir.

Genellikle şu sensörler kullanılır:

* Wheel odometry (teker encoder)
* IMU

EKF bu verileri birleştirerek robot için daha stabil bir **odometry çıktısı** üretir.

Bu bilgi navigasyon sistemi tarafından kullanılır.

---

## EKF Parametre Dosyası

EKF node’un davranışı bir YAML dosyası ile belirlenir.

Örnek node:

```
ekf_filter_node_odom
```

### Güncelleme frekansı

```yaml
frequency: 30.0
```

EKF node’un saniyede kaç kez çalışacağını belirler.

---

### 2D Mode

```yaml
two_d_mode: true
```

Robot sadece düz zeminde hareket ettiği için:

* z ekseni
* roll
* pitch

hesaplamaları kapatılır.

---

### TF yayını

```yaml
publish_tf: true
```

EKF robot için şu dönüşümü yayınlar:

```
odom → base_footprint
```

---

# ROS2 Navigasyonunda TF Zinciri

Navigasyon sisteminde robotun konumu farklı koordinat sistemleri ile tanımlanır.

Tipik TF zinciri şu şekildedir:

```
map → odom → base_footprint → base_link → sensors
```

Her frame robot sisteminde farklı bir anlam taşır.

---

# map Frame

```
map
```

Robotun **harita koordinat sistemidir.**

Bu frame genellikle:

* SLAM
* AMCL

tarafından yayınlanır.

---

# odom Frame

```
map → odom
```

`odom` frame robotun kısa süreli hareketini temsil eder.

Bu dönüşüm genellikle:

* SLAM
* AMCL

tarafından üretilir.

Odometry zamanla hata biriktirebilir.
Bu hata `map` frame ile düzeltilir.

---

# base_footprint Frame

```
odom → base_footprint
```

Robotun **zemin üzerindeki konumunu** temsil eder.

Bu frame:

* x
* y
* yaw

hareketlerini içerir.

---

# base_link Frame

```
base_footprint → base_link
```

Robotun gövdesini temsil eder.

Sensörler genellikle bu frame’e göre tanımlanır.

---

# Sensors Frame

```
base_link → sensors
```

Robot üzerindeki sensörlerin kendi koordinat sistemleri vardır.

Örneğin:

```
base_link → lidar
base_link → imu
base_link → camera
```

Bu dönüşümler genellikle robot modelinde tanımlanır.

---

# SLAM (Harita Oluşturma)

Robot bulunduğu ortamın haritasını çıkarmak için **SLAM** kullanır.

Bu projede kullanılan SLAM parametre dosyası:

```
matrobot_slam/config/mapper_params_online_sync.yaml
```

SLAM sistemi:

* LiDAR verisini kullanır
* robot hareket ederken harita oluşturur
* `map` topic’i yayınlar

Bu harita daha sonra navigasyon sistemi tarafından kullanılır.

---

# Harita Kaydetme

SLAM ile oluşturulan harita kalıcı olarak kaydedilebilir.

Haritayı kaydetmek için şu komut kullanılır:

```
ros2 run nav2_map_server map_saver_cli -f my_map
```

Bu komut iki dosya oluşturur:

```
my_map.pgm
my_map.yaml
```

Bu dosyalar daha sonra navigasyon sisteminde kullanılabilir.

---

# Nav2 (Navigasyon Sistemi)

Nav2 robotun belirlenen bir hedef noktaya **otonom olarak gitmesini sağlar.**

Nav2 şu işlemleri gerçekleştirir:

* hedefe giden yolu planlar
* engellerden kaçınır
* robotun yolu takip etmesini sağlar

Nav2 parametre dosyası:

```
matrobot_navigation/config/nav2_params.yaml
```

Bu dosyada robotun:

* hız limitleri
* costmap ayarları
* planlama parametreleri

bulunur.

---

# Sistem Çalıştırma Sırası

Genellikle sistem şu sırayla çalıştırılır.

### 1) Simülasyon veya robot başlatılır

Robot veya Gazebo ortamı çalıştırılır.

---

### 2) EKF başlatılır

Robot localization node çalıştırılır.

---

### 3) SLAM başlatılır

Robot ortamın haritasını çıkarır.

---

### 4) Harita kaydedilir

```
ros2 run nav2_map_server map_saver_cli -f my_map
```

---

### 5) Nav2 başlatılır

Kaydedilen harita kullanılarak navigasyon sistemi başlatılır.

---

# Navigasyon Sisteminde Sık Karşılaşılan Problemler

### Robot haritada yanlış yerde görünür

Genellikle TF zinciri hatalıdır.

Kontrol edilmesi gereken:

```
map → odom → base_footprint → base_link
```

---

### LiDAR harita ile hizalanmaz

Genellikle sensör transformu yanlış tanımlanmıştır.

---

### Nav2 plan üretir fakat robot hareket etmez

Sebebi genellikle:

* `/cmd_vel` gönderilmiyordur
* controller çalışmıyordur
* hız limitleri çok düşük olabilir.

---

### Costmap sürekli engel ile doludur

Sebebi genellikle sensör parametrelerinin yanlış ayarlanmasıdır.

---

# Özet

Bu bölümde:

* EKF node’un sensör verilerini birleştirerek robot konumunu hesapladığını
* TF zincirinin navigasyon sistemindeki rolünü
* SLAM ile harita oluşturmayı
* Nav2 ile otonom navigasyonu

inceledik.

Bu üç sistem birlikte çalışarak robotun **otonom hareket etmesini sağlar.**
