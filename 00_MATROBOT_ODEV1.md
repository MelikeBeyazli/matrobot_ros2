# 📘 ROS2 MATROBOT PARAMETRE ANALİZ ÖDEVİ

Repo:
🔗 [https://github.com/MelikeBeyazli/matrobot_ros2.git](https://github.com/MelikeBeyazli/matrobot_ros2.git)

---

# 🎯 ÖDEVİN AMACI

Bu ödevde:

* Terminal kullanmayı öğreneceksiniz
* ROS2 workspace oluşturacaksınız
* Robot tanım parametrelerini değiştireceksiniz
* Joint limit, velocity, damping gibi kavramları test edeceksiniz
* Gazebo fizik parametrelerini değiştireceksiniz
* World dosyasının simülasyona etkisini göreceksiniz
* Sensörlerin sistem plugin mantığını anlayacaksınız

Bu ödev **parametre değiştir → test et → gözlemle** mantığı üzerine kuruludur.

Not almak zorunlu değildir, ancak tüm testler yapılmalıdır.

---

# 1️⃣ WSL Ubuntu Dosya Yönetimi (Windows Kullananlar İçin)

Ubuntu kullanıyorsanız bu bölümü geçebilirsiniz.

WSL kullanıyorsanız dosyaları VSCode ile açmanız önerilir.

📎 Kurulum rehberi:
🔗 [https://github.com/MelikeBeyazli/matrobot_ros2/blob/main/images/Daha%20kolay%20dosya%20düzenlemek%20için%20VSCODE-WSL%20kurulumu.pdf](https://github.com/MelikeBeyazli/matrobot_ros2/blob/main/images/Daha%20kolay%20dosya%20düzenlemek%20için%20VSCODE-WSL%20kurulumu.pdf)

---

# 2️⃣ WORKSPACE OLUŞTURMA

Terminal açın:

```bash
cd ~
mkdir -p matro_ws/src
cd matro_ws/src
```

> `~` = `/home/kullanıcı_adı`

---

# 3️⃣ REPOYU İNDİRME

```bash
git clone https://github.com/MelikeBeyazli/matrobot_ros2.git
```

---

# 4️⃣ DERLEME

```bash
cd ~/matro_ws
colcon build
source install/setup.bash
```

---

# 5️⃣ TEMEL TERMINAL KOMUT PRATİĞİ

Aşağıdakileri deneyerek neyin ne olduğunu güzelce kavrayalım:

```bash
cd ~
ls
mkdir test
cd test
pwd
cd ..
rm -r test
touch deneme.txt
nano deneme.txt
```

---

# 🧠 BÖLÜM 1 — URDF / XACRO PARAMETRE TESTİ

Dosya:

```
matrobot_description/urdf/matrobot.xacro
```

Bu dosya robotun **geometrik ve fiziksel tanımıdır**.

---

## 🔵 AŞAMA 1 — Geometrik Parametre

Değiştirilebilecek örnekler:

* wheel_radius
* base_width
* box size
* origin xyz

Build + test:

```bash
colcon build
source install/setup.bash
ros2 launch urdf_tutorial display.launch.py model:=<xacro_yolu>
```

Robotun boyutu değişti mi?

---

## 🔵 AŞAMA 2 — Joint Parametreleri

Bulun:

```xml
<limit lower="" upper="" effort="" velocity=""/>
<dynamics damping="" friction=""/>
```

Şunlardan 1 tanesini değiştirin:

* lower
* upper
* velocity
* damping
* friction

Tekrar build + test edin.

Eklem daha hızlı mı?
Dönüş sınırı değişti mi?
Daha yumuşak mı hareket etti?

---

## 🔴 GAZEBO’YA GEÇMEDEN ÖNCE

⚠️ Tüm değişiklikleri eski haline getirin.

```bash
colcon build
source install/setup.bash
```

---

# 🟢 BÖLÜM 2 — GAZEBO (XACRO İÇİNDEKİ PARAMETRELER)

⚠️ Simülasyon testlerinde iki terminal kullanın.

Terminal 1:

```bash
ros2 launch matrobot_simulation simulation.launch.py
```

Terminal 2:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 🟢 AŞAMA 3 — Surface (Gerçek Fizik Sürtünmesi)

Xacro içinde tekerlek linkinde bulun:

```xml
<surface>
  <friction>
    <ode>
      <mu>...</mu>
      <mu2>...</mu2>
```

Bu kısım **gerçek fizik sürtünmesidir**.

`mu` veya `mu2` değiştirin.

Robot daha mı kaygan oldu?

---

## 🟢 AŞAMA 4 — Gazebo Override (mu1 / mu2)

Xacro içinde:

```xml
<gazebo reference="...">
  <mu1>1.5</mu1>
  <mu2>1.5</mu2>
</gazebo>
```

Bu Gazebo’ya özel sürtünme override’ıdır.

`mu1` ve `mu2` değiştirin.

Surface ile farkı karşılaştırın.

---

# 🌍 BÖLÜM 3 — WORLD DOSYASI PARAMETRE TESTİ

World dosyası:

```
matrobot_simulation/worlds/
```

---

## 🌍 AŞAMA 1 — Gravity

World dosyasında `<gravity>` değerini değiştirin.

* Robot daha hafif mi?
* Daha mı yavaş düşüyor?

---

## 🌍 AŞAMA 2 — Physics Parametreleri

World dosyasında `<physics>` bloğunu bulun.

Şunlardan 1 tanesini değiştirin:

* step size
* real time factor
* solver iteration

Simülasyon stabilitesi değişti mi?

---

## 🌍 AŞAMA 3 — Sensör Sistem Plugin Testi (World)

Sensör tanımı xacro’da yapılmıştır.
Ancak world dosyasında şu sistem plugin’leri vardır:

```xml
<plugin filename="gz-sim-sensors-system"
        name="gz::sim::systems::Sensors"/>

<plugin filename="gz-sim-imu-system"
        name="gz::sim::systems::Imu"/>
```

Bu iki plugin’i silin.

Build + çalıştırın.

```bash
ros2 topic echo /scan
ros2 topic echo /imu/data
```

Veri kesildi mi?

Plugin’leri geri koyun ve tekrar test edin.

📌 Sonuç:
Sensör tanımı olsa bile sistem plugin yoksa çalışmayabilir.

---

# 🟣 BÖLÜM 4 — Sensör Parametre Testi (XACRO)

`matrobot.xacro` içinde <sensor> etiketi içerinde senor yapıalrımız bulunmaktadır. Öreneğin lidar sensör bloğunu bulun:

```xml
<sensor name="rplidar_s1" type="gpu_lidar">
```

Şunlardan 1 tanesini değiştirin:

* samples
* update_rate
* min_angle
* max_angle
* max range

Simülasyonu başlatın.

Robot önüne engel koyun.

Kontrol:

```bash
ros2 topic echo /scan
```

Değişiklik gözlemlendi mi?

---

# 📌 SON MESAJ

Bu ödevin amacı:

* Robot tanımı (URDF/Xacro)
* Gazebo parametreleri
* World fiziği
* Sensör sistemi

arasındaki farkı deneyerek öğrenmektir.

Her değişiklikten sonra mutlaka test edin.

---
