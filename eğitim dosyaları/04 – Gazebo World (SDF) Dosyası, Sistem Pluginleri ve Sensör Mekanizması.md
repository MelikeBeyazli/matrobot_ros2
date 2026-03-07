# 04 – Gazebo World (SDF) Dosyası, Sistem Pluginleri ve Sensör Mekanizması

Robotun kendisi URDF/XACRO ile tanımlanır.
**Fakat dünya (World)** ve **simülasyon motoru** SDF formatı ile yönetilir.

Bu dosya:

* Fizik motorunu,
* Sensör sistemlerini,
* Render motorunu,
* Simülasyonun GUI arayüzünü,
* Ortam modellerini,
* Işık kaynaklarını,
* Hazır Fuel objelerini,

yönetir.

**URDF yalnızca robotu tanımlar. Dünya çalışmazsa sensör verisi de gelmez, robot hareket etmez.**

---

# 1️⃣ SDF WORLD NEDİR?

SDF (Simulation Description Format) Gazebo’nun ana dosya formatıdır.

Bir world (dünya) dosyası:

* Yerçekimi
* Fizik motoru
* Ortam modelleri
* Aydınlatma
* GUI
* Sensör altyapısı
* Dünya pluginleri

gibi *tüm sahne yönetimini* içerir.

**Robot → URDF/XACRO**
**Dünya → SDF**

---

# 2️⃣ World Dosyasının Genel Yapısı

Bu yapıyı anlamak çok önemlidir:
**Sensörler yalnızca URDF içinde tanımlamakla çalışmaz!**

Gazebo’nun sensör sistem plugin’i world içinde aktif edilmelidir:

```xml
<plugin filename="gz-sim-sensors-system"
        name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

Bu plugin etkin değilse:

* Lidar veri üretmez
* Kamera görüntü vermez
* IMU çalışmaz

---

# 3️⃣ WORLD İÇİNDEKİ ANA PLUGINLER (SDF)

Aşağıdaki pluginler *URDF içinde değil; WORLD içinde bulunmak zorundadır* çünkü sensör hesaplamaları dünya düzeyinde yapılır.

---

## 🔹 **3.1 Physics System Plugin**

```xml
<plugin filename="gz-sim-physics-system"
        name="gz::sim::systems::Physics"/>
```

Bu plugin şunları sağlar:

* Yerçekimi
* Çarpışmalar
* Kütle ve atalet hesapları
* Hareket simülasyonu
* Adım zamanlayıcısı

Bu plugin *olmazsa sim motoru çalışmaz!*.

---

## 🔹 **3.2 User Commands Plugin**

Gazebo GUI üzerinden:

* Model ekleme
* Model silme
* Objeleri taşıma
* Interaktif kullanım

için gereklidir.

---

## 🔹 **3.3 Scene Broadcaster Plugin**

```xml
<plugin filename="gz-sim-scene-broadcaster-system"
        name="gz::sim::systems::SceneBroadcaster"/>
```

Gazebo Client ile Server arasında sahne senkronizasyonu sağlar.

Eğer yoksa:

* Görüntü güncellenmez
* Kamera hareketleri görünmez

---

# 4️⃣ Sensör Sistemi: Gazebo’nun Çalışması İçin EN KRİTİK KISIM

## ✔ **4.1 Sensors System Plugin (Lidar, Kamera, Derinlik)**

```xml
<plugin filename="gz-sim-sensors-system"
        name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

Bu plugin dünya düzeyinde devreye girer.

**URDF’de sensor tanımlamak → Ham tanım yapar**
**Sensors-system plugin → Sensörleri hesaplayan motoru çalıştırır**

Bu plugin olmadığında:

* `/scan` boş gelir
* `/camera` görüntü üretmez
* Derinlik sensörü çalışmaz

---

## ✔ 4.2 IMU Sistem Plugin’i

```xml
<plugin filename="gz-sim-imu-system"
        name="gz::sim::systems::Imu"/>
```

URDF’de IMU olsa bile:
👉 Bu plugin yoksa IMU verisi üretilmez.

IMU hesaplaması:

* Açısal hız
* Lineer ivme
* Gürültü modeli
* Bias drift

gibi işlemleri dünya motoru içinde yapar.

---

# 5️⃣ GUI Pluginleri (Dünya Arayüzünü Oluşturan Bileşenler)

## ✔ 3D View (Sahne Görüntüsü)

Render motoru (`ogre2`) ile sahneyi görüntüler.

## ✔ World Control

Simülasyonu oynatma / durdurma / adım adım ilerletme.

## ✔ World Stats

FPS, real-time factor, sim-time, iterations gibi metrikleri gösterir.

## ✔ Teleop Plugin

`/cmd_vel` üzerinden robotu klavye ile kontrol etmeni sağlar.

---

---

# 6️⃣ Dünya Ortamı Oluşturma ve Fuel Modeller

World dosyasında Fuel modellerini doğrudan referans gösterebilirsin:

```xml
<include>
  <uri>https://fuel.gazebosim.org/.../Construction Cone</uri>
</include>
```

Avantajları:

* Hazır objeler
* Doğru fizik parametreleri
* Realistik simülasyon
* Öğrenci için hızlı başlangıç

---

# 7️⃣ Zemin (Ground Plane) Yapısı

Her world mutlaka bir zemin modeline sahip olmalıdır:

```xml
<model name='ground_plane'>
  <static>true</static>
  <link name='link'>
    <collision>
      <geometry>
        <plane>...</plane>
      </geometry>
    </collision>
  </link>
</model>
```

Zemin yoksa:

* Robot yere düşer
* Sensörler “sonsuz boşluk” görür

---

# 8️⃣ SDF → URDF Robot Dahil Etme Mantığı

Robot world içine şu şekilde eklenir:

```xml
export GZ_SIM_RESOURCE_PATH=:$HOME/robot_path/models
```

```xml
<include>
  <uri>model://matrobot</uri>
</include>
```

veya launch dosyasından spawn edilir.

---

# 9️⃣ Bu Bölümün Sonunda Öğrenci Şunları Öğrenir:

✔ Neden yalnızca URDF ile sensör verisi gelmediğini
✔ World dosyasının simülasyonun merkezi olduğunu
✔ Physics / Sensors / IMU sistemlerinin çalışma biçimini
✔ OGRE2 render motorunun neden tercih edildiğini
✔ Gazebo GUI pluginlerinin ne işe yaradığını
✔ Fuel modellerinin nasıl kullanıldığını

---
