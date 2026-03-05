# 09 – ROS2 Node Mantığı: Manuel Kontrol ve Basit Otonom Davranış

Bu bölümde ROS2 içinde bir robotu kontrol etmek için **node yazma mantığını** inceleyeceğiz.

Robot kontrolünü iki farklı şekilde ele alacağız:

1. **Manuel kontrol (Teleop)**
   Robot klavye ile kontrol edilir.

2. **Basit otonom kontrol**
   Robot LiDAR verisini kullanarak önünde engel görürse yön değiştirir.

Bu iki örnek sayesinde ROS2’de:

* Node yapısı
* Publisher
* Subscriber
* Callback
* Timer

gibi temel kavramlar anlaşılacaktır.

---

# ROS2 Node Nedir?

ROS2’de **node**, çalışan bir programdır.

Her node belirli bir görevi yerine getirir ve diğer node’lar ile **topic** üzerinden haberleşir.

Örneğin robotu klavye ile kontrol eden bir node şu şekilde çalışır:

```
Keyboard
   │
   ▼
Teleop Node
   │
   │ publish
   ▼
 /cmd_vel
   │
   ▼
Robot hareketi
```

Bu yapıda klavyeden gelen komutlar `/cmd_vel` topic’i üzerinden robota gönderilir.

---

# Publisher Nedir?

Publisher bir topic’e **mesaj gönderen bileşendir.**

Robotu hareket ettirmek için kullanılan topic:

```
/cmd_vel
```

Bu topic robotun hız komutlarını taşır.

---

# Twist Mesajı

Robot hareket komutları **Twist** mesajı ile gönderilir.

Mesajın en önemli alanları şunlardır:

```
linear.x   → ileri / geri hareket
angular.z  → sağ / sol dönüş
```

Örnek bir hareket komutu:

```
linear.x = 1.0
angular.z = 0.0
```

Robot ileri doğru hareket eder.

---

# Subscriber Nedir?

Subscriber bir topic’i **dinleyen bileşendir.**

Örneğin robotun LiDAR sensörü aşağıdaki topic üzerinden veri gönderir:

```
/scan
```

Bu topic şu mesaj tipini kullanır:

```
sensor_msgs/msg/LaserScan
```

Bu mesaj robotun çevresindeki mesafeleri içerir.

---

# Callback Nedir?

ROS2’de bir subscriber mesaj aldığında **otomatik olarak bir fonksiyon çalıştırılır.**

Bu fonksiyona **callback** denir.

Çalışma mantığı şu şekildedir:

```
mesaj geldi
↓
callback fonksiyonu çalıştı
↓
veri işlendi
```

Örneğin `/scan` topic’inden bir LiDAR mesajı geldiğinde callback fonksiyonu çalışır ve sensör verisi okunur.

---

# Timer Nedir?

Bazı işlemler belirli aralıklarla tekrar edilir.

Bu işlemler için **timer** kullanılır.

Örneğin robot kontrol döngüsü şu şekilde çalışabilir:

```
her 100 ms
↓
kontrol fonksiyonu çalışır
↓
cmd_vel mesajı gönderilir
```

Bu yöntem sürekli `while` döngüsü kullanmaktan daha güvenlidir.

---

# Manuel Kontrol: Teleop Node

Teleop node robotu **klavye ile kontrol etmek** için kullanılır.

Bu node:

1. Klavye girdisini okur
2. Robot hızını hesaplar
3. `/cmd_vel` topic’ine hız komutu gönderir

Veri akışı şu şekildedir:

```
Keyboard
   │
   ▼
Teleop Node
   │
   ▼
 /cmd_vel
   │
   ▼
Robot
```

---

# Klavye Kontrolleri

Robot yön tuşları ile kontrol edilir:

```
↑  → ileri git
↓  → geri git
←  → sola dön
→  → sağa dön
SPACE → hız artır
```

SPACE tuşu basılıyken robot daha hızlı hareket eder.

---

# Teleop Node Çalışma Mantığı

Teleop node belirli aralıklarla klavye durumunu kontrol eder.

Bu işlem şu şekilde gerçekleşir:

```
klavye kontrol edilir
↓
hız hesaplanır
↓
Twist mesajı oluşturulur
↓
/cmd_vel topic’ine yayınlanır
```

Bu işlem sürekli tekrar edilir.

---

# Pygame Kullanımı

Klavye girdisini almak için **pygame** kütüphanesi kullanılır.

Pygame şu işlemleri sağlar:

* klavye tuşlarını algılama
* tuşların basılı olup olmadığını kontrol etme
* küçük bir görsel arayüz oluşturma

Arayüz üzerinde robotun hız bilgileri gösterilebilir.

---

# Otonom Kontrol: Basit Engel Kaçınma

Robot sensör verilerini kullanarak kendi başına hareket edebilir.

Bu örnekte robot **LiDAR verisini kullanarak engelden kaçacaktır.**

Veri akışı şu şekildedir:

```
/scan (LiDAR)
      │
      ▼
 Avoid Node
      │
      ▼
  /cmd_vel
      │
      ▼
   Robot
```

---

# LiDAR Verisi Nasıl Kullanılır?

LiDAR sensörü robotun etrafındaki mesafeleri ölçer.

Bu veriler `/scan` topic’i üzerinden gönderilir.

Node şu işlemi yapar:

1. `/scan` verisini dinler
2. robotun önündeki mesafeyi kontrol eder
3. engel olup olmadığına karar verir

---

# Engel Algılama Mantığı

Robotun önünde belirli bir mesafeden daha yakın bir nesne varsa engel olduğu kabul edilir.

Basit bir mantık:

```
engel yok → ileri git
engel var → sağa dön
```

Bu sayede robot engelden kaçabilir.

---

# Callback ve Kontrol Döngüsü

Engel algılama sisteminde iki farklı işlem vardır.

### Sensör verisini almak

```
/scan mesajı geldi
↓
callback çalıştı
↓
engel bilgisi güncellendi
```

### Robot hareketini kontrol etmek

```
timer çalıştı
↓
engel var mı kontrol edildi
↓
cmd_vel mesajı gönderildi
```

Bu yapı sensör verisi ile kontrol sisteminin daha stabil çalışmasını sağlar.

---

# Node Yazarken Dikkat Edilmesi Gerekenler

### Tek görev prensibi

Her node mümkün olduğunca tek bir görev yapmalıdır.

Örnek:

```
teleop node → manuel kontrol
avoid node → sensör analizi
```

---

### Topic isimleri standart olmalıdır

ROS sistemlerinde bazı topic isimleri yaygın olarak kullanılır.

```
/cmd_vel
/scan
/tf
/clock
```

---

### Aynı topic’e birden fazla publisher yazılmamalıdır

Birden fazla node aynı anda `/cmd_vel` yayınlarsa robot kararsız hareket edebilir.

---

### Callback fonksiyonları hızlı olmalıdır

Callback fonksiyonları mümkün olduğunca kısa tutulmalıdır.

Callback içinde ağır hesaplamalar yapılmamalıdır.

---

### Robot güvenli şekilde durdurulmalıdır

Node kapanırken robotun durması gerekir.

Bu nedenle genellikle şu komut gönderilir:

```
linear.x = 0
angular.z = 0
```

---

# Node Test Komutları

ROS2 içinde node’ları ve topic’leri kontrol etmek için bazı komutlar kullanılır.

Topicleri görmek:

```
ros2 topic list
```

Bir topic’in mesajlarını görmek:

```
ros2 topic echo /cmd_vel
```

Bir topic hakkında bilgi almak:

```
ros2 topic info /cmd_vel
```

---

# Bu Bölümde Öğrenilenler

Bu bölümde:

* ROS2 node yapısı
* Publisher ve Subscriber mantığı
* Callback fonksiyonları
* Timer tabanlı kontrol döngüsü
* `/cmd_vel` ile robot kontrolü
* LiDAR verisi ile basit otonom davranış

incelenmiştir.

---
