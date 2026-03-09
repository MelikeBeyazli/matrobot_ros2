## 1) ROS ortamını hazırla

Önce ROS 2 dağıtımının sistemde kurulu olduğundan emin ol. Bu projede örnek olarak `jazzy` kullanılıyor.

Terminal açıp ortamı yükle:

```bash
export ROS_DISTRO=jazzy
source /opt/ros/$ROS_DISTRO/setup.bash
```

İstersen bunu kalıcı yapmak için `~/.bashrc` dosyana ekleyebilirsin:

```bash
# ROS Dağıtımı
export ROS_DISTRO=jazzy

# ROS Domain ID
export ROS_DOMAIN_ID=22

# ROS 2 ortamını yükle
source /opt/ros/$ROS_DISTRO/setup.bash

# Workspace'leri yükle
if [ -f ~/matro_ws/install/setup.bash ]; then
  source ~/matro_ws/install/setup.bash
fi

if [ -f ~/pusula_ws/install/setup.bash ]; then
  source ~/pusula_ws/install/setup.bash
fi

if [ -f ~/kangal_ws/install/setup.bash ]; then
  source ~/kangal_ws/install/setup.bash
fi

if [ -f ~/hexapod_ws/install/setup.bash ]; then
  source ~/hexapod_ws/install/setup.bash
fi

if [ -f ~/zemheri_ws/install/setup.bash ]; then
  source ~/zemheri_ws/install/setup.bash
fi

alias build='colcon build && source ~/.bashrc'
```

Not: `systemd` servisleri `~/.bashrc` dosyasını otomatik okumaz. Bu yüzden servis içinde ROS ortamı ayrıca tanımlanır. Aşağıda bunun kurulumu da var.

---

## 2) Workspace oluşturma

Terminal açın:

```bash
cd ~
mkdir -p matro_ws/src
cd ~/matro_ws/src
```

> `~` = `/home/kullanıcı_adı`

---

## 3) Repoyu indirme

```bash
git clone https://github.com/MelikeBeyazli/matrobot_ros2.git
```

---

## 4) Derleme

```bash
cd ~/matro_ws
colcon build
source install/setup.bash
```

Paketin düzgün derlendiğini kontrol etmek için:

```bash
ros2 pkg list | grep matrobot_bringup
```

---

## 5) Joystick yönetim sistemi

Bu projede joystick ile aşağıdaki işlemler yapılır:

* `joy_node` sürekli açık kalır
* `A` tuşu ile `matrobot_bringup/bringup.launch.py` açılır veya kapanır
* `B` tuşu ile `teleop_cmd_vel.launch.py` açılır veya kapanır
* `X` tuşu ile `matrobot_slam/slam_async.launch.py use_rviz:=true` açılır veya kapanır

Sistem açıldıktan sonra çalışan ana launch:

```bash
ros2 launch matrobot_bringup joystick_manager.launch.py
```

---

## 6) Joystick autostart script oluşturma

Açılışta otomatik başlatmak için script oluştur:

```bash
nano ~/matro_ws/src/matrobot_ros2/start_matrobot_joystick.sh
```

İçine şunu yaz:

```bash
#!/bin/bash

export ROS_DISTRO=jazzy
export ROS_DOMAIN_ID=22

source /opt/ros/$ROS_DISTRO/setup.bash
source /home/mb/matro_ws/install/setup.bash

exec ros2 launch matrobot_bringup joystick_manager.launch.py
```

Sonra çalıştırılabilir yap:

```bash
chmod +x ~/matro_ws/src/matrobot_ros2/start_matrobot_joystick.sh
```

---

## 7) Systemd servis dosyasını oluşturma

Servis dosyasını oluştur:

```bash
sudo nano /etc/systemd/system/matrobot_joystick.service
```

İçine şunu yaz:

```ini
[Unit]
Description=MatRobot Joystick Manager
After=network.target bluetooth.target
Wants=bluetooth.target

[Service]
Type=simple
User=mb
Environment=HOME=/home/mb
Environment=RCUTILS_LOGGING_BUFFERED_STREAM=1
WorkingDirectory=/home/mb
ExecStart=/home/mb/matro_ws/src/matrobot_ros2/start_matrobot_joystick.sh
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
```

---

## 8) Servisi aktifleştirme

Servis dosyasını sisteme tanıt:

```bash
sudo systemctl daemon-reload
```

Otomatik başlatmayı aç:

```bash
sudo systemctl enable matrobot_joystick.service
```

Servisi başlat:

```bash
sudo systemctl start matrobot_joystick.service
```

---

## 9) Servis durumunu kontrol etme

Servisin çalıştığını kontrol et:

```bash
systemctl status matrobot_joystick.service
```

Beklenen çıktı içinde şuna benzer bir satır olmalı:

```bash
Active: active (running)
```

Canlı log izlemek için:

```bash
journalctl -u matrobot_joystick.service -f
```

---

## 10) Çalışan node'ları kontrol etme

```bash
ros2 node list
```

Beklenen node'lar:

```bash
/joy_node
/joy_multi_toggle
```

---

## 11) Tuş işlevleri

Joystick bağlıyken:

* `A` → `bringup.launch.py` aç/kapat
* `B` → `teleop_cmd_vel.launch.py` aç/kapat
* `X` → `slam_async.launch.py use_rviz:=true` aç/kapat

---

## 12) Faydalı servis komutları

Servisi durdurmak için:

```bash
sudo systemctl stop matrobot_joystick.service
```

Servisi yeniden başlatmak için:

```bash
sudo systemctl restart matrobot_joystick.service
```

Servisi devre dışı bırakmak için:

```bash
sudo systemctl disable matrobot_joystick.service
```

---

## 13) Yeniden başlatma testi

Sistemi yeniden başlat:

```bash
sudo reboot
```

Açıldıktan sonra servis durumunu kontrol et:

```bash
systemctl status matrobot_joystick.service
```

---

## 14) Sistem akışı

```text
Bilgisayar açılır
   ↓
systemd servisi başlar
   ↓
start_matrobot_joystick.sh çalışır
   ↓
ros2 launch matrobot_bringup joystick_manager.launch.py
   ↓
joy_node + joy_multi_toggle başlar
   ↓
Joystick tuşları ile launch dosyaları açılır / kapanır
```
