#!/usr/bin/env python3

# ROS2 python kütüphanesi
import rclpy
from rclpy.node import Node

# Robot hız komutlarını göndermek için kullanılan mesaj tipi
# Twist mesajı robotun lineer (ileri/geri) ve açısal (dönme) hızını içerir
from geometry_msgs.msg import Twist

# Klavye kontrolü ve basit grafik arayüz için pygame
import pygame
import sys


class KeyboardTeleop(Node):

    def __init__(self):
        # ROS2 node oluşturuyoruz
        super().__init__('keyboard_teleop')

        # Robotu hareket ettirmek için /cmd_vel topicine mesaj gönderen publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Robotun maksimum lineer hızı (m/s)
        # Aracın max hızı 0.56 olduğu için bu değer kullanılıyor
        self.base_speed = 0.56

        # Robotun dönme hızı (rad/s)
        self.angular_speed_val = 1.5

        # Her 0.1 saniyede bir loop fonksiyonu çalışır
        self.timer = self.create_timer(0.1, self.loop)

        # -------- Pygame arayüz başlatma --------
        pygame.init()

        # 400x400 boyutunda pencere oluşturulur
        self.screen = pygame.display.set_mode((400, 400))
        pygame.display.set_caption("Keyboard Teleop 🚗")

        # Yazı fontu
        self.font = pygame.font.SysFont("Arial", 24)

        # FPS kontrolü için clock
        self.clock = pygame.time.Clock()

        self.get_logger().info("Keyboard teleop başlatıldı. Ok tuşları ile robotu kontrol et.")


    def loop(self):

        # Pencere kapatılırsa program tamamen kapanır
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                rclpy.shutdown()
                sys.exit(0)

        # Basılı tuşları kontrol eder
        keys = pygame.key.get_pressed()

        # Robot hareket mesajı oluşturulur
        twist = Twist()

        # ---------------------------------------------------
        # LINEAR HAREKET (ileri / geri)
        # twist.linear.x robotun ileri-geri hızını belirler
        # ---------------------------------------------------
        if keys[pygame.K_UP]:
            twist.linear.x = self.base_speed     # ileri git

        elif keys[pygame.K_DOWN]:
            twist.linear.x = -self.base_speed    # geri git

        else:
            twist.linear.x = 0.0                 # dur


        # ---------------------------------------------------
        # ANGULAR HAREKET (dönme)
        # twist.angular.z robotun sağa/sola dönmesini sağlar
        # ---------------------------------------------------
        if keys[pygame.K_LEFT]:
            twist.angular.z = -self.angular_speed_val   # sola dön

        elif keys[pygame.K_RIGHT]:
            twist.angular.z = self.angular_speed_val    # sağa dön

        else:
            twist.angular.z = 0.0                       # dönme yok


        # Robot hız mesajını /cmd_vel topicine gönderiyoruz
        self.cmd_vel_pub.publish(twist)


        # -------- Basit görsel arayüz --------

        # Arka plan rengi (RGB)
        self.screen.fill((25, 25, 25))

        # Robot hızını ekrana yazdır
        lin_text = self.font.render(f"Linear: {twist.linear.x:.2f} m/s", True, (255, 255, 255))
        ang_text = self.font.render(f"Angular: {twist.angular.z:.2f} rad/s", True, (255, 255, 255))

        self.screen.blit(lin_text, (20, 20))
        self.screen.blit(ang_text, (20, 60))


        # -------- Robot yön göstergesi --------

        center = (200, 250)      # robotu temsil eden dairenin merkezi
        arrow_color = (0, 200, 255)

        # Robotu temsil eden daire çiziyoruz
        # pygame.draw.circle(ekran, renk, merkez, yarıçap, çizgi_kalınlığı)
        pygame.draw.circle(self.screen, (100, 100, 100), center, 60, 3)


        # -------- İleri / geri ok çizimi --------
        # pygame.draw.polygon üçgen oluşturur (3 nokta ile)

        if twist.linear.x > 0.0:  # ileri hareket
            pygame.draw.polygon(self.screen, arrow_color,
                                [(200, 210), (180, 250), (220, 250)])

        elif twist.linear.x < 0.0:  # geri hareket
            pygame.draw.polygon(self.screen, arrow_color,
                                [(200, 290), (180, 250), (220, 250)])


        # -------- Dönüş göstergesi --------
        # pygame.draw.arc robotun dönmesini göstermek için yay çizer

        if twist.angular.z != 0.0:
            pygame.draw.arc(
                self.screen,
                (255, 100, 100),
                (130, 180, 140, 140),     # yay çizilecek alan
                0 if twist.angular.z > 0 else 3.14,
                3.14 if twist.angular.z > 0 else 6.28,
                5
            )

        # Ekranı güncelle
        pygame.display.flip()

        # FPS sınırı (saniyede 30 frame)
        self.clock.tick(30)



def main(args=None):

    # ROS2 başlat
    rclpy.init(args=args)

    # Node oluştur
    node = KeyboardTeleop()

    try:
        # Node sürekli çalışır
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Teleop kapatılıyor...")

    finally:
        # Program kapanırken temizlik yapılır
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()


# Dosya doğrudan çalıştırılırsa main fonksiyonu başlar
if __name__ == '__main__':
    main()
