# 06 – SolidWorks URDF Modelini ROS2 Paketinde Kullanma

Bu derste SolidWorks URDF Exporter ile oluşturduğumuz robot modelini
ROS2 içinde test edeceğiz.

Amaç:

```

SolidWorks Modeli
↓
URDF Export
↓
ROS2 içinde görüntüleme (RViz)

```

Bu aşamada modelimizi **Gazebo simülasyonuna eklemeden önce**
RViz içinde kontrol edeceğiz.

---

# 1) Export Edilen Dosyaları Paketimize Eklemek

SolidWorks URDF Exporter robot modeli için genelde şu klasörü oluşturur:

```

robot_package
│
├── meshes
│
└── urdf
└── robot.urdf

```

Burada bulunan dosyalar:

- **meshes/** → Robot parçalarının STL modelleri
- **robot.urdf** → Robotun URDF tanımı

Bu dosyaları kendi ROS2 paketimize kopyalarız.

Örnek paket yapısı:

```

matrobot_description
│
├── meshes
│   |
│   ├── base_link.STL
│   ├── left_wheel.STL
│   ├── right_wheel.STL
│   └── caster_wheel.STL
│
└── urdf
    |
    └── matrobot_sw.urdf

```

Export edilen `robot.urdf` dosyasını

```

matrobot_description/urdf/matrobot_sw.urdf

```

olarak kaydediyoruz.

---

# 2) Mesh Dosya Yollarını Düzeltmek

SolidWorks export edilen URDF içinde mesh yolları bazen şöyle olabilir:

```

meshes/base_link.STL

```

veya

```

file:///C:/... (düzenle bunu sonradan)

````

ROS2 içinde doğru kullanım:

```xml
<mesh filename="package://matrobot_description/meshes/solidworks/base_link.STL"/>
````

Buradaki

```
package://
```

ifadesi ROS'a şunu söyler:

> Bu dosya robot paketinin içindeki meshes klasöründe bulunuyor.

Bu yüzden URDF içindeki tüm mesh yollarını **package:// formatına** çevirmek gerekir.

---

# 3) Collision ve Visual Geometrisi

SolidWorks modelinde genellikle robot parçaları **mesh** olarak gelir.

Simülasyon performansını artırmak için şu yaklaşım kullanılabilir:

* **Visual** → Mesh kullanılır (gerçek görünüm)
* **Collision** → Basit geometri kullanılır (box / cylinder)

Örnek:

```xml
<link name="base_link">

  <visual>
    <geometry>
      <mesh filename="package://matrobot_description/meshes/solidworks/base_link.STL"/>
    </geometry>
  </visual>

  <collision>
    <geometry>
      <box size="0.40 0.34 0.11"/>
    </geometry>
  </collision>

</link>
```

Bu yöntem sayesinde:

* Robot **gerçek CAD modeli gibi görünür**
* Simülasyon **daha hızlı çalışır**

---

# 4) Inertia Değerleri

SolidWorks URDF Exporter robot parçaları için inertia değerlerini de üretir.

Örnek:

```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="5.0"/>
  <inertia
    ixx="0.02"
    ixy="0"
    ixz="0"
    iyy="0.03"
    iyz="0"
    izz="0.04"/>
</inertial>
```

Bu değerler **SolidWorks tarafından hesaplanan gerçek kütle dağılımıdır.**

Bu yüzden inertia değerlerini değiştirmeden kullanabilirsiniz.

Önerilen yapı:

```
Visual   → mesh
Collision → basit geometri
Inertia  → SolidWorks export değerleri
```

---

# 5) URDF Modelini ROS2 ile Test Etme

URDF modelimizi hızlıca test etmek için
`urdf_tutorial` paketinin `display.launch.py` dosyasını kullanacağız.

Bu launch dosyası:

* robot_state_publisher
* joint_state_publisher_gui
* RViz

başlatır ve robot modelini görüntüler.

---

## urdf_tutorial Paketini Kontrol Et

Eğer sisteminizde kurulu değilse:

```bash
sudo apt install ros-${ROS_DISTRO}-urdf-tutorial
```

---

# 6) URDF Modelini Çalıştırmak

URDF modelini test etmek için şu komutu çalıştırın:

```bash
ros2 launch urdf_tutorial display.launch.py model:=<urdf_yolu>
```
---

# 7) RViz İçinde Kontrol

Komut çalıştırıldığında RViz açılacaktır.

Burada şu kontroller yapılabilir:

* Robot modeli doğru görünüyor mu
* Parçalar doğru konumda mı
* Tekerlekler doğru yerde mi
* LiDAR ve IMU linkleri doğru yerde mi

`joint_state_publisher_gui` sayesinde eklemleri de hareket ettirip
robot modelinin davranışını gözlemleyebilirsiniz.

---
