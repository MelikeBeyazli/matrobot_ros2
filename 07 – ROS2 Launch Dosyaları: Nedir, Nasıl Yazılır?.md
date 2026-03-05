# 07 – ROS2 Launch Dosyaları: Nedir, Nasıl Yazılır?

Bu derste ROS2’de kullandığımız **launch dosyalarının** ne olduğunu ve bir launch dosyası yazarken en sık kullanılan yapıları öğreneceğiz.

Launch dosyası kısaca şunu yapar:

> Tek komutla birden fazla node’u başlatır, parametrelerini verir, istersen bazılarını koşula göre açar/kapatır ve başka launch dosyalarını da dahil edebilir.

Bu sayede:
- aynı sistem her seferinde aynı şekilde açılır
- terminalde 10 komut yazmak yerine 1 komut yeter
- simülasyon/gerçek robot gibi farklı senaryolar için parametreyle kontrol sağlanır

---

## 1) ROS2’de Launch Dosyası Mantığı

ROS2 Python launch dosyalarının genel şablonu şöyledir:

```python
#!/usr/bin/env python3

from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        # burada actions / node’lar yer alır
    ])

* `generate_launch_description()` fonksiyonu çalışır
* içindeki listeye yazdığımız her şey sırayla launch edilir

---

## 2) Launch Argument (Parametre) Tanımlama

Launch dosyasında dışarıdan parametre almak için `DeclareLaunchArgument` kullanırız.

Örnek: `use_sim_time` ve `use_rviz`

```python
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

use_sim_time = LaunchConfiguration('use_sim_time')
use_rviz = LaunchConfiguration('use_rviz')

declare_sim_time = DeclareLaunchArgument(
    'use_sim_time',
    default_value='true',
    description='Use simulation time'
)

declare_rviz = DeclareLaunchArgument(
    'use_rviz',
    default_value='true',
    description='Launch RViz'
)
```

Bu sayede terminalden şöyle kontrol edebiliriz:

```bash
ros2 launch <paket> <launch>.launch.py use_rviz:=false
```

---

## 3) Hazır Node Çalıştırma (Node Action)

Bir node’u launch dosyasıyla başlatmak için `Node()` kullanırız.

Örnek: `robot_state_publisher`

```python
from launch_ros.actions import Node

robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[
        {'use_sim_time': use_sim_time},
        {'robot_description': robot_description}
    ],
)
```

Burada önemli alanlar:

* `package`: node’un bulunduğu paket
* `executable`: çalıştırılacak executable
* `name`: node adı
* `parameters`: parametreler (yaml veya dictionary)
* `output`: ekran çıktısı

---

## 4) Koşullu Çalıştırma (If / Unless)

Bazı node’lar her durumda çalışmasın isteyebiliriz.
Bunun için `IfCondition` ve `UnlessCondition` kullanılır.

Örnek: RViz sadece `use_rviz:=true` ise açılsın:

```python
from launch.conditions import IfCondition

rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    condition=IfCondition(use_rviz),
)
```

Örnek: `joint_state_publisher`, sadece sim time kapalıysa çalışsın:

```python
from launch.conditions import UnlessCondition

joint_state_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    condition=UnlessCondition(use_sim_time),
)
```

---

## 5) Paket İçinden Dosya Yolu Bulma

Launch dosyasında “paketin içindeki” bir dosyaya ulaşmak için:

```python
from ament_index_python.packages import get_package_share_directory

pkg_path = get_package_share_directory('matrobot_description')
```

Sonra `os.path.join()` ile dosya yolu yapılır:

```python
import os

xacro_path = os.path.join(pkg_path, 'urdf', 'matrobot.xacro')
```

---

## 6) Xacro Çalıştırıp robot_description Üretme

`robot_state_publisher` node’u robot modelini `robot_description` parametresinden okur.

Xacro kullanıyorsak, launch dosyasında xacro’yu çalıştırıp string üretiriz:

```python
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

robot_description = ParameterValue(
    Command([
        'xacro ',
        xacro_path,
        ' use_gazebo:=', use_sim_time
    ]),
    value_type=str
)
```

Bu sayede:

* xacro çalışır
* URDF çıktısı alınır
* `robot_description` parametresi olarak yayınlanır

---

## 7) YAML Parametre Dosyası Kullanma

Node parametrelerini yaml dosyasından da verebiliriz:

```python
hardware_config = os.path.join(pkg_share, 'config', 'hardware_params.yaml')

hardware_node = Node(
    package='matrobot_hardware',
    executable='HardwareNode',
    parameters=[
        hardware_config,
        {'serial_port': serial_port}
    ]
)
```

Bu yöntemle:

* büyük parametre setleri düzenli olur
* tek tek python içinde yazmak zorunda kalmayız

---

## 8) Başka Launch Dosyasını Dahil Etme (IncludeLaunchDescription)

Hazır bir launch dosyasını çağırmak için `IncludeLaunchDescription` kullanırız.

Örnek: IMU driver paketi kendi launch dosyasını sağlar, biz onu dahil ederiz:

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

imu_driver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('witmotion_hwt905_driver'),
            'launch',
            'imu.launch.py'
        )
    )
)
```

Bu sayede:

* başka paketin launch yapısını tekrar yazmayız
* “hazır launch” direkt kullanılır

---

## 9) Gerçek Robot İçin: Donanım + LiDAR + IMU Launch Örneği

Bu launch dosyası şunları başlatır:

* donanım node (Arduino/motor sürüş)
* IMU driver (Include)
* RPLidar node

Buradaki önemli örnekler:

* `DeclareLaunchArgument` ile port seçimi
* `parameters` ile serial ayarı
* hazır launch include etme

---

## 10) Lokalizasyon İçin: EKF Launch Örneği

Bu launch dosyası:

* `robot_localization` paketindeki `ekf_node`’u başlatır
* yaml dosyasını seçilebilir yapar (`ekf_yaml`)

Örnek olarak:

* `PathJoinSubstitution`
* `PythonExpression` ile yaml adı seçme
  kullanılıyor.

Ama temel fikir basit:

> EKF node’u başlat ve konfigürasyonu yaml’dan yükle.

---

## 11) Simülasyon Launch Mantığı (Gazebo’ya Giriş)

Bir sim launch dosyasında genelde şu parçalar olur:

1. **Robot description** (xacro → robot_description → RViz/TF)
2. **Gazebo’yu açma**
3. **Robotu Gazebo’ya spawn etme**
4. **ROS <-> Gazebo Bridge**
5. (İstersen) **Localization / EKF gibi ek sistemler**

Örnekte gördüğümüz yapı:

* `ros_gz_sim` ile Gazebo başlatılıyor
* `ros_gz_sim create` ile robot spawn ediliyor
* `ros_gz_bridge parameter_bridge` ile topicler köprüleniyor
* localization launch ayrıca include ediliyor

> Bu dersin sonunda Gazebo detayına girmiyoruz.
> Gazebo kısmını bir sonraki derste adım adım işlenecek şekilde ayırıyoruz.

---

## Bu Derste Ne Öğrendik?

* Launch dosyası nedir ve neden kullanılır?
* Launch argument (parametre) tanımlama
* Node çalıştırma
* Koşullu node çalıştırma (if/unless)
* Paket içi dosya yolu bulma
* Xacro’yu launch içinde çalıştırıp robot_description üretme
* YAML parametre dosyası kullanma
* Hazır launch dosyalarını include etme
* Sim launch dosyalarının genel mantığı

---

## Next Topic →

Sonraki derslerde:

1. **GZ Bridge (ROS2 ↔ Gazebo) ayrı ders**
2. Simülasyonda node yazımı ve test (ör: cmd_vel testleri, topic kontrol, TF kontrol)

```
```
