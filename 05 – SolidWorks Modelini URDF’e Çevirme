# 05 – SolidWorks Modelini URDF’e Çevirme (SW2URDF)

Bu derste SolidWorks’te tasarlanan bir robot modelini **URDF formatına export ederek ROS ortamında kullanmayı** öğreneceğiz.

Bu bölüm özellikle **mekanik tasarım yapan ekip arkadaşları** için hazırlanmıştır.

Amaç:

```
SolidWorks CAD Modeli
        ↓
URDF Export
        ↓
ROS2 (RViz / Gazebo)
```

---

# 1) SolidWorks → URDF Exporter Eklentisini Kur

SolidWorks modelini URDF formatına çevirmek için **SolidWorks URDF Exporter** eklentisini kullanacağız.

ROS Wiki sayfası:

[https://wiki.ros.org/sw_urdf_exporter](https://wiki.ros.org/sw_urdf_exporter)

---

## 1.1 SolidWorks Versiyonunu Kontrol Et

Öncelikle kullandığınız **SolidWorks versiyonunu** kontrol edin.

URDF Exporter sürümleri SolidWorks versiyonuna göre değişebilir.

---

## 1.2 URDF Exporter İndir

Bu eğitimde **SolidWorks 2021** kullandığımız için **v1.6.1** sürümünü indiriyoruz.

Adımlar:

1. ROS Wiki sayfasında **Download Installer** butonuna tıkla
2. GitHub release sayfası açılacaktır
3. **v1.6.1 (SolidWorks 2021)** sürümünü bul
4. **sw2urdfSetup.exe** dosyasını indir
5. sw2urdfSetup.exe` dosyasına sağ tıkla
6. **Run as administrator**
7. Kurulum ekranında **Next** ile devam et


<p align="center">
<img src="images/sw_urdf.jpg" width="900">
</p>
---

## 1.4 SolidWorks İçinde Eklentiyi Aktif Et

Kurulumdan sonra eklentiyi SolidWorks içinde aktif etmemiz gerekir.

Adımlar:

1. SolidWorks aç
2. **Tools → Add-ins**
3. **URDF Exporter** eklentisini bul
4. Hem **Active Add-ins** hem **Start-Up** seçeneklerini işaretle

<p align="center">
<img src="images/sw_addins_enable.jpg" width="700">
</p>

---

# 2) ROS Coordinate System Standardı

URDF oluştururken coordinate system'lerin **ROS standardına uygun olması gerekir**.

ROS coordinate system standardı:

```
X → forward
Y → left
Z → up
```

Detaylı bilgi:

[https://www.ros.org/reps/rep-0103.html](https://www.ros.org/reps/rep-0103.html)

<p align="center">
<img src="images/ros_coordinate_standard.png" width="500">
</p>

---

# 3) Base Link Coordinate System Oluşturma

Robotun ana referans noktası **base_link** olacaktır.

### Adımlar

1. **Reference Geometry → Center of Mass**
2. Robot gövdesinin **Center of Mass** noktasını bulun
3. **Reference Geometry → Coordinate System**
4. Coordinate system'i ROS standardına göre hizalayın

Axis yönleri:

```
X → robotun ileri yönü
Y → robotun sol tarafı
Z → yukarı
```

<p align="center">
<img src="images/base_link_coordinate.jpg" width="700">
</p>

---

# 4) Wheel Coordinate System Oluşturma

Her tekerlek için ayrı bir coordinate system tanımlanmalıdır.

### Adımlar

1. **Reference Geometry → Point**
2. Wheel merkezine bir nokta oluştur
3. **Reference Geometry → Coordinate System**
4. Origin olarak oluşturduğun point'i seç

<p align="center">
<img src="images/wheel_center_point.jpg" width="700">
</p>

---

## Wheel Rotation Axis

Wheel dönüş ekseni doğru seçilmelidir.

Genellikle:

* Wheel axle
* Wheel cylinder surface

kullanılır.

<p align="center">
<img src="images/wheel_axis_selection.jpg" width="700">
</p>

---

# 5) Sensörler İçin Coordinate System

Bu robot üzerinde iki sensör bulunmaktadır:

* **LiDAR**
* **IMU**

Bu sensörlerin coordinate system'lerinin doğru tanımlanması gerekmektedir.

---

# 5.1 LiDAR Coordinate System

LiDAR sensörü çevrenin **mesafe ölçümünü** yapar ve **LaserScan verisi üretir**.

### Adımlar

1. **Reference Geometry → Point**
2. LiDAR sensörünün merkezine bir nokta oluştur
3. **Reference Geometry → Coordinate System**
4. Origin olarak bu noktayı seç

Axis yönleri:

```
X → forward
Y → left
Z → up
```

<p align="center">
<img src="images/lidar_coordinate_system.jpg" width="700">
</p>

URDF içinde bu frame genellikle şu isimle kullanılır:

```
lidar_link
```

---

# 5.2 IMU Coordinate System

IMU sensörü robotun:

* acceleration
* angular velocity
* orientation

bilgilerini ölçer.

### Adımlar

1. IMU sensörünün merkezine bir **Reference Point** oluştur
2. **Reference Geometry → Coordinate System**
3. Origin olarak bu noktayı seç

Axis yönleri yine ROS standardına göre ayarlanmalıdır.

```
X → forward
Y → left
Z → up
```

<p align="center">
<img src="images/imu_coordinate_system.jpg" width="700">
</p>

URDF içinde bu frame genellikle şu isimle kullanılır:

```
imu_link
```

---

# 6) URDF Export

Artık robot modelimizi URDF olarak export edebiliriz.

### Adımlar

1. SolidWorks → **Tools**
2. **URDF Exporter**
3. Robotun **base link**'ini seç
4. Child linkleri ekle
5. Wheel ve sensör linklerini ekle

<p align="center">
<img src="images/urdf_exporter_window.jpg" width="700">
</p>

---

# 7) Link ve Joint Yapısı

Robotun URDF link ağacı şu şekilde olacaktır:

```
base_link
 ├── left_wheel_link
 ├── right_wheel_link
 ├── lidar_link
 └── imu_link
```

<p align="center">
<img src="images/urdf_link_tree.jpg" width="700">
</p>

---

# 8) URDF Preview ve Export

Export işleminden önce robot modelini kontrol edebilirsiniz.

### Adımlar

1. **Preview and Export**
2. Modeli kontrol et
3. Eğer her şey doğruysa export et

<p align="center">
<img src="images/urdf_preview.jpg" width="700">
</p>

---


<p align="center">
<img src="images/exported_package_structure.png" width="600">
</p>

Bu klasör içinde robot modeline ait tüm gerekli dosyalar bulunur.

Özellikle önemli olanlar:

- **meshes/** → Robotun 3D modelleri (STL / DAE)
- **urdf/robot.urdf** → Robotun URDF tanımı
- **launch/** → Simülasyon veya görüntüleme launch dosyaları

---

# URDF Dosyasını Hızlı Test Etme

URDF dosyasını ROS çalıştırmadan önce hızlıca test etmek için **online URDF viewer** araçlarını kullanabilirsiniz.

Örnek bir araç:

https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/

Bu site sayesinde URDF modelinizi doğrudan tarayıcı içinde görüntüleyebilirsiniz.

### Test Adımları

1. Siteyi açın
2. Export ettiğiniz klasörü bulun
3. Klasörü siteye **sürükleyip bırakın (drag & drop)**

Site robot modelini otomatik olarak yükleyip 3D olarak gösterecektir.

Bu yöntem sayesinde şu kontrolleri hızlıca yapabilirsiniz:

- Mesh dosyaları doğru yükleniyor mu
- Robot parçaları doğru hizalanmış mı
- Link yapısı doğru mu
- Ölçek doğru mu

Bu kontrol özellikle **mekanik tasarım yapan ekip için çok faydalıdır**, çünkü ROS ortamını kurmadan önce modelin doğru export edilip edilmediği hızlıca doğrulanabilir.
---
