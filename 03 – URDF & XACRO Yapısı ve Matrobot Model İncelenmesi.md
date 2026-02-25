# 03 – URDF - XACRO YAPISI ve MATROBOT MODEL İNCELEMESİ

Bu bölümde önce **genelden özele** bir yaklaşım izlenecek:

1️⃣ URDF’in genel yapısı ve fiziksel modelleme mantığı
2️⃣ Xacro’nun URDF’i nasıl modüler hâle getirdiği
3️⃣ Link–Joint–Inertia–Collision–Visual–Sensor tag’lerinin anlamı
4️⃣ Gazebo’ya özel `surface`, `friction`, `sensor`, `plugin` yapılandırmaları
5️⃣ Matrobot modelinin satır satır analizi

---

# 1️⃣ URDF Nedir? (Genel Yapı)

URDF (Unified Robot Description Format), robotların **fiziksel ve kinematik yapısını tanımlayan XML tabanlı bir formattır.**

Bir URDF dosyası tipik olarak aşağıdaki bileşenlerden oluşur:

## ✔ Link

Robotun katı (rigid) parçalarıdır.

Her link 3 önemli alt bileşen içerir:

* **inertial** → kütle + atalet
* **collision** → fiziksel çarpışma modeli
* **visual** → görsel model

---

## ✔ Joint

Link’leri birbirine bağlar.
Robotun hareket özgürlüklerini belirler.

Temel joint türleri:

| Joint Tipi     | Açıklama                      |
| -------------- | ----------------------------- |
| **fixed**      | Hareket yok                   |
| **continuous** | Sonsuz dönüş (teker)          |
| **revolute**   | Belirli açılar arasında dönme |
| **prismatic**  | Doğrusal hareket              |
| **floating**   | 6 serbestlik (drone)          |
| **planar**     | 2D hareket                    |

Her joint mutlaka:

* parent link
* child link
* origin
* axis

bilgilerine sahiptir.

---

## ✔ Collision vs Visual

### 🔹 Visual

* Kullanıcıya görünen modeldir
* Detaylı mesh (.stl/.dae) kullanılabilir
* Render odaklıdır

### 🔹 Collision

* Fizik motorunun kullandığı gerçek çarpışma hacmidir
* **Basit geometri** önerilir (box, cylinder, sphere)
* Detaylı mesh kullanılması → simülasyon *yavaşlar*

---

## ✔ Inertial (Kütle + Atalet)

Fiziğin en kritik parçasıdır:

* Kütle (mass)
* Kütle merkezi (origin)
* Atalet tensörü (inertia matrix)

Yanlış inertia → robot gerçeksiz davranır, sensörler bozulur, SLAM ve Nav2 çöker.

---

## ✔ Gazebo’ya Özel Ekler

URDF, robotu tanımlar.
GZ (Gazebo) ise robotun fiziksel davranışını yönetir.

Gazebo özel tag’leri:

* `<gazebo>` → model ayarlarının tamamı
* `<surface>` → sürtünme (mu, mu2), restitüsyon
* `<sensor>` → lidar, imu, kamera
* `<plugin>` → hareket, eklem kontrolü, state publisher

Bu tag’ler URDF’in fiziksel doğruluğunu *çok* artırır.

---

# 🎨 URDF Yapısının Görsel Şeması

Aşağıdaki görseller, URDF yapısını görsel olarak anlamayı kolaylaştırır:

---

# 2️⃣ Xacro Nedir? URDF’i Nasıl Güçlendirir?

Xacro, URDF'i **daha esnek, modüler ve hesaplanabilir** yapar.

Xacro ile:

* Makrolar oluşturabilir
* Parametre tanımlayabilir
* Hesaplamalar yapabilir `${...}`
* Tekrarlayan yapıları sadeleştirebilirsin

Xacro → Büyük robot projelerinde *zorunlu* hâle gelir.

---

# 3️⃣ Gazebo İçin Özel Fizik Yapıları

(GENEL ROBOT TÜRLERİ İÇİN)

URDF’deki collision tek başına yeterli değildir.
Gazebo gerçekçi fizik için ek parametrelere ihtiyaç duyar.

---

## ✔ 3.1 Surface → Sürtünme (Friction)

```xml
<surface>
  <friction>
    <ode>
      <mu>1.5</mu>
      <mu2>1.5</mu2>
    </ode>
  </friction>
</surface>
```

**mu** → ileri-geri sürtünme
**mu2** → yan sürtünme

Tekerli robotlarda doğru sürtünme olmazsa:

* Robot kayar
* Frenleyemez
* Nav2 kontrolü bozulur

Manipülatörlerde:

* Kavrama yüzeyleri yanlış hesaplanır

Dronlarda:

* Pratik olarak kullanılmaz

Su üstü/altı robotlarda:

* Bunun yerine “drag coefficients” kullanılır.

---

## ✔ 3.2 Damping & friction (Joint içinde)

```xml
<dynamics damping="0.1" friction="0.01"/>
```

Damping:

* Eklem hareketlerini yumuşatır
* Kontrol salınımını azaltır

Joint friction:

* Gerçekçi sürtünme ekler

Bu parametreler özellikle:

* Teker motorlarında
* Manipülatör eklemlerinde
* Bacaklı robotlarda

çok önemlidir.

---

## ✔ 3.3 Update Rate

Sensörlerin hesaplanma hızını belirler.

Yüksek update rate → daha gerçekçi ama daha fazla CPU.

---
# 4️⃣ Matrobot Xacro Dosyasının Ayrıntılı İncelemesi

Aşağıda modelin *tamamı* teknik olarak açıklanmıştır.

---

## 4.1 Dosya Başlığı

```xml
<robot name="matrobot" xmlns:xacro="http://www.ros.org/wiki/xacro">
```

✔ Xacro dosyası
✔ Robot adı: matrobot

---

## 4.2 Property Tanımları (Parametreler)

Bu bölüm robotun tüm boyutlarını merkezi olarak yönetir:

```xml
<xacro:property name="base_width" value="0.34" />
<xacro:property name="wheel_radius" value="0.09" />
...
```

Bu yapı:

* Modülerlik
* Hızlı değişiklik
* Hataların kolay tespiti

sağlar.

---

## 4.3 Base Link + Inertia

```xml
<link name="base_link">
  <xacro:box_inertia ... />
```

✔ Gövdenin kütlesi
✔ Atalet tensörü
✔ Collision → kutu
✔ Visual → kutu + renk

Gövde, robotun “en ağır” bileşenidir → inertia kritik.

---

## 4.4 Teker Makrosu

```xml
<xacro:macro name="Wheel_Link">
```

Her teker:

* Link
* Collision (silindir)
* Visual
* Sürtünme
* Joint (continuous)

içerir.

Bu sayede:

✔ Simde gerçekçi hareket
✔ Yanal kaymanın engellenmesi
✔ Hızlı dönüşlerde stabilite

sağlanır.

---

## 4.5 Caster Wheel

Destek tekeridir; motorlu değildir.

Collision → küre
Visual → küre
Joint → fixed

Robotun ağırlık merkezini destekler.

---

## 4.6 Lidar Link + Sensor

```xml
<sensor type="gpu_lidar">
```

✔ 360°
✔ 640 örnek
✔ 40 m menzil

Bu lidar ayarları:

* SLAM doğruluğunu
* Navigasyon kalitesini

doğrudan etkiler.

---

## 4.7 IMU Sensörü

Sensör gürültü parametrelerinin kulalnılan imuya göre düzenlenebilir.

* bias
* stddev
* correlation time

→ EKF çıktısını GERÇEK robot gibi yapar.

---

## 4.8 Gazebo Visual Settings

Materyal ve sürtünme ayarları:

```xml
<xacro:gazebo_visual_settings ... />
```

* Renk
* Sürtünme
* Fizik materyali

Bu kısım simin doğruluğunu artırır.

---

## 4.9 DiffDrive Plugin (En Kritik Kısım)

```xml
<plugin filename="gz-sim-diff-drive-system">
```

Bu plugin:

* `/cmd_vel` → teker hızlarına çevirir
* Odometri üretir
* wheel separation & radius kullanır
* Gerçek hareket modelini verir

Bu plugin olmazsa robot **hareket etmez**.

---

## 4.10 Joint State Publisher Plugin

RViz ve Navigation için gereklidir.

---

# 🎨 URDF – Gazebo Bağlantılarını Gösteren Görsel

---

# 5️⃣ Bu Bölümün Kazanımları

Bu bölümü tamamlayan katılımcı:

* URDF/Xacro yapısını genel robotik perspektiften anlar
* Inertia’nın tüm robot türlerindeki önemini kavrar
* Collision/Visual mantığını bilir
* Gazebo fizik parametrelerini yorumlayabilir
* Matrobot’un URDF/Xacro yapısını baştan sona anlayabilir
* Yeni robot modelleri oluşturabilecek seviyeye gelir

---
