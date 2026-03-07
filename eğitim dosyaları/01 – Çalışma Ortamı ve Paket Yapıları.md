# 01 – ÇALIŞMA ORTAMI VE PAKET YAPILARI

Bu bölümde:

* Projeye özel çalışma ortamı oluşturma
* ROS2 workspace yapısı
* Derleme süreci
* `build / install / log` klasörleri
* `.bashrc` yapılandırması
* ROS_DOMAIN_ID
* ROS_DISTRO export
* Workspace source işlemleri
* Modüler paket mimarisi
anlatılacaktır.

---

# 1️⃣ Neden Projeye Özel Çalışma Ortamı?

Birden fazla robot projesiyle çalışıyorsanız:

* Paket çakışmaları oluşabilir
* Farklı ROS sürümleri gerekebilir
* Bağımlılıklar karışabilir
* Domain ID çakışması olabilir

Bu nedenle her proje için ayrı workspace oluşturulması önerilir.

Örnek:

```bash
~/matrobot_ws
~/agv_ws
~/drone_ws
```

Bu yapı:

* İzolasyon sağlar
* Daha temiz derleme ortamı sunar
* Ekip içi yönetimi kolaylaştırır

---

# 2️⃣ ROS2 Workspace Yapısı

Workspace oluşturma:

```bash
mkdir -p ~/matrobot_ws/src
cd ~/matrobot_ws
```

Workspace yapısı:

```
matrobot_ws/
 ├── src/
 ├── build/
 ├── install/
 └── log/
```

---

## 📂 src

Kaynak kodların bulunduğu klasördür.

Tüm ROS2 paketleri burada yer alır.

---

## 📂 build

Derleme sırasında oluşturulur.

* Geçici derleme dosyaları
* CMake ara çıktıları
* Object dosyaları

Silinebilir.

---

## 📂 install

Derlenmiş paketlerin çalıştırılabilir hali burada bulunur.

* setup.bash burada oluşur
* ROS2 paket yolu buraya eklenir
* Node’lar buradan çalışır

Asıl aktif klasör burasıdır.

---

## 📂 log

Derleme hataları ve çıktı logları burada tutulur.

Hata ayıklamada kullanılır.

---

# 3️⃣ Derleme Süreci

ROS2’de derleme aracı:

```
colcon
```

Derleme:

```bash
colcon build
```

Belirli paket:

```bash
colcon build --packages-select matrobot_description
```

Temiz derleme:

```bash
colcon build --cmake-clean-cache
```

---

# 4️⃣ Workspace Source Etme

Derleme sonrası:

```bash
source install/setup.bash
```

Bu işlem yapılmazsa:

* Paketler bulunamaz
* Node’lar çalışmaz
* Launch dosyaları hata verir

Her terminal açıldığında tekrar source edilmelidir.

---

# 5️⃣ ~/.bashrc Yapılandırması

Sürekli source etmeyi önlemek için `.bashrc` içine eklenir.

Aç:

```bash
nano ~/.bashrc
```

---

## 📌 ROS Distro Export

```bash
export ROS_DISTRO=jazzy
source /opt/ros/$ROS_DISTRO/setup.bash
```

### Neden export ediyoruz?

* ROS sürümü değiştiğinde tek yerden değiştirilebilir
* Ekip içinde sürüm standardı sağlanır
* Paket indirirken distro uyumu korunur

---

## 📌 Workspace Source

```bash
source ~/matrobot_ws/install/setup.bash
```

Yeni terminal açıldığında otomatik yüklenir.

---

## 📌 ROS_DOMAIN_ID

```bash
export ROS_DOMAIN_ID=7
```

---

### ROS_DOMAIN_ID Nedir?

ROS2 DDS tabanlıdır.

Aynı ağda birden fazla robot varsa:

* Topic karışmasını önler
* Robotları izole eder
* Laboratuvar ortamında çakışmayı engeller

Her robot için farklı Domain ID kullanılabilir.

Örnek:

| Robot     | Domain |
| --------- | ------ |
| matrobot  | 7      |
| agv_robot | 8      |

---

## 📌 ROS Middleware (RMW) – *(İleri Seviye Dipnot)*

> ⚠️ Bu bölüm ileri seviye kullanıcılar içindir. Eğitim sırasında zorunlu değildir.

ROS2, doğrudan bir haberleşme sistemi kullanmaz.
Alt katmanda **DDS (Data Distribution Service)** çalışır.

Bu katman ile ROS2 arasında bulunan soyutlama yapısına:

```
RMW (ROS Middleware Interface)
```

denir.

### 📌 RMW Ne İşe Yarar?

RMW, ROS2'nin hangi DDS altyapısını kullanacağını belirler.

Örneğin:

| RMW                | Açıklama              |
| ------------------ | --------------------- |
| rmw_fastrtps_cpp   | Varsayılan (Fast DDS) |
| rmw_cyclonedds_cpp | Cyclone DDS           |
| rmw_connextdds     | RTI Connext           |

---

### 📌 Neden Değiştirilir?

* Ağ keşif (discovery) sorunları varsa
* Çok robotlu sistemlerde performans problemi varsa
* Endüstriyel projelerde özel DDS gereksinimi varsa

---

### 📌 Nasıl Ayarlanır?

`.bashrc` içine şu şekilde eklenebilir:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Alternatif:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

# 6️⃣ ROS2 Paket Oluşturma

Artık `src` klasörüne geçiyoruz.

```bash
cd ~/matrobot_ws/src
```

---

# 7️⃣ Modüler Paket Yapısı

Tek bir paket yerine modüler yapı tercih edilir.

Neden?

* Bakımı kolaydır
* Hata izolasyonu sağlar
* Takım çalışmasına uygundur
* Derleme süresi azalır
* Büyük projelerde ölçeklenebilirlik sağlar

---

## 📦 Matrobot Paket Yapısı

```
matrobot_description
matrobot_bringup
matrobot_hardware
matrobot_simulation
matrobot_slam
matrobot_navigation
```

---

## 7.1 matrobot_description

* URDF
* Xacro
* Mesh dosyaları

Robotun fiziksel tanımı.

---

## 7.2 matrobot_bringup

* Launch dosyaları
* Parametre yükleme
* Robot başlatma

---

## 7.3 matrobot_hardware

* Gerçek sensör sürücüleri
* Encoder verileri
* IMU sürücüsü
* Donanım arayüzü

---

## 7.4 matrobot_simulation

* Gazebo launch dosyaları
* World dosyaları
* Plugin ayarları

---

## 7.5 matrobot_slam

* SLAM konfigürasyonu
* slam_toolbox parametreleri

---

## 7.6 matrobot_navigation

* Nav2 yapılandırması
* EKF ayarları
* Costmap ayarları

---

# 8️⃣ Paket Oluşturma

Örnek:

```bash
ros2 pkg create matrobot_description --build-type ament_cmake
```

Python paket için:

```bash
ros2 pkg create matrobot_bringup --build-type ament_python
```

---

# 9️⃣ CMakeLists.txt Nedir?

Derleme yapılandırma dosyasıdır.

İçerir:

* Bağımlılıklar
* Include klasörleri
* Executable tanımları
* Install talimatları

Önemli bölümler:

```
find_package(rclcpp REQUIRED)
ament_target_dependencies()
install()
```

---

# 🔟 package.xml Nedir?

Paket metadata dosyasıdır.

İçerir:

* Paket adı
* Versiyon
* Lisans
* Bağımlılıklar

Önemli etiketler:

```
<depend>rclcpp</depend>
<build_depend>
<exec_depend>
```

Bağımlılık tanımlamazsanız:

* Derleme hatası
* Runtime hatası oluşur

---

# 11️⃣ Bu Bölümün Kazanımları

Bu bölüm sonunda katılımcı:

* Projeye özel workspace oluşturabilir
* build/install/log yapısını bilir
* .bashrc yapılandırabilir
* ROS_DOMAIN_ID mantığını anlar
* Modüler paket yapısı kurabilir
* CMakeLists ve package.xml görevini bilir
