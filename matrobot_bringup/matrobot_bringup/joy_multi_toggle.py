#!/usr/bin/env python3

import os
import shlex
import signal
import subprocess
from datetime import datetime
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class ManagedLaunch:
    def __init__(self, name: str, command: List[str], logger):
        self.name = name
        self.command = command
        self.logger = logger
        self.process: Optional[subprocess.Popen] = None
        self.pgid: Optional[int] = None

    def is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def start(self) -> bool:
        if not self.command:
            self.logger.warning(f"{self.name} için komut tanımlı değil.")
            return False

        if self.is_running():
            self.logger.info(f"{self.name} zaten çalışıyor.")
            return True

        try:
            self.logger.info(f"{self.name} başlatılıyor: {' '.join(self.command)}")
            self.process = subprocess.Popen(
                self.command,
                preexec_fn=os.setsid
            )
            self.pgid = os.getpgid(self.process.pid)
            self.logger.info(
                f"{self.name} başladı. PID={self.process.pid}, PGID={self.pgid}"
            )
            return True
        except Exception as e:
            self.logger.error(f"{self.name} başlatılamadı: {e}")
            self.process = None
            self.pgid = None
            return False

    def stop(self) -> bool:
        if self.process is None:
            self.logger.info(f"{self.name} için aktif process yok.")
            return True

        try:
            if self.process.poll() is not None:
                self.logger.info(f"{self.name} zaten kapanmış.")
                return True

            self.logger.info(f"{self.name} launch grubu kapatılıyor...")

            if self.pgid is not None:
                os.killpg(self.pgid, signal.SIGINT)
            else:
                self.process.send_signal(signal.SIGINT)

            self.process.wait(timeout=8.0)
            self.logger.info(f"{self.name} düzgün şekilde kapandı.")
            return True

        except subprocess.TimeoutExpired:
            self.logger.warning(
                f"{self.name} SIGINT sonrası kapanmadı, SIGTERM deneniyor..."
            )
            try:
                if self.pgid is not None:
                    os.killpg(self.pgid, signal.SIGTERM)
                else:
                    self.process.terminate()

                self.process.wait(timeout=4.0)
                self.logger.info(f"{self.name} SIGTERM ile kapandı.")
                return True
            except subprocess.TimeoutExpired:
                self.logger.warning(
                    f"{self.name} SIGTERM sonrası da kapanmadı, SIGKILL uygulanıyor..."
                )
                try:
                    if self.pgid is not None:
                        os.killpg(self.pgid, signal.SIGKILL)
                    else:
                        self.process.kill()

                    self.process.wait(timeout=2.0)
                    self.logger.warning(f"{self.name} SIGKILL ile kapatıldı.")
                    return True
                except Exception as kill_e:
                    self.logger.error(f"{self.name} kill hatası: {kill_e}")
                    return False
            except Exception as e:
                self.logger.error(f"{self.name} SIGTERM hatası: {e}")
                return False
        except Exception as e:
            self.logger.error(f"{self.name} durdurma hatası: {e}")
            return False
        finally:
            self.process = None
            self.pgid = None

    def toggle(self) -> bool:
        if self.is_running():
            return self.stop()
        return self.start()


class JoyMultiToggle(Node):
    def __init__(self):
        super().__init__('joy_multi_toggle')

        self.declare_parameter('button_a_index', 0)
        self.declare_parameter('button_b_index', 1)
        self.declare_parameter('button_x_index', 2)  # map save
        self.declare_parameter('button_y_index', 3)  # command_y
        self.declare_parameter('debounce_sec', 0.35)

        self.declare_parameter('command_a', '')
        self.declare_parameter('command_b', '')
        self.declare_parameter('command_y', '')

        self.declare_parameter(
            'map_save_root',
            '~/matrobot_ws/src/matrobot_ros2/matrobot_navigation/maps'
        )
        self.declare_parameter(
            'map_save_command',
            'ros2 run nav2_map_server map_saver_cli'
        )

        self.button_a_index = int(self.get_parameter('button_a_index').value)
        self.button_b_index = int(self.get_parameter('button_b_index').value)
        self.button_x_index = int(self.get_parameter('button_x_index').value)
        self.button_y_index = int(self.get_parameter('button_y_index').value)
        self.debounce_sec = float(self.get_parameter('debounce_sec').value)

        command_a_str = str(self.get_parameter('command_a').value).strip()
        command_b_str = str(self.get_parameter('command_b').value).strip()
        command_y_str = str(self.get_parameter('command_y').value).strip()

        self.map_save_root = os.path.expanduser(
            str(self.get_parameter('map_save_root').value)
        )
        self.map_save_command_str = str(
            self.get_parameter('map_save_command').value
        ).strip()

        command_a = shlex.split(command_a_str) if command_a_str else []
        command_b = shlex.split(command_b_str) if command_b_str else []
        command_y = shlex.split(command_y_str) if command_y_str else []

        self.launches: Dict[str, ManagedLaunch] = {
            'A': ManagedLaunch('launch_a', command_a, self.get_logger()),
            'B': ManagedLaunch('launch_b', command_b, self.get_logger()),
            'Y': ManagedLaunch('launch_y', command_y, self.get_logger()),
        }

        self.button_map = {
            'A': self.button_a_index,
            'B': self.button_b_index,
            'X': self.button_x_index,
            'Y': self.button_y_index,
        }

        self.prev_states = {'A': 0, 'B': 0, 'X': 0, 'Y': 0}
        self.last_press_times = {'A': 0.0, 'B': 0.0, 'X': 0.0, 'Y': 0.0}

        self.joy_ready = False
        self.first_joy_received = False

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.readiness_timer = self.create_timer(2.0, self.check_readiness)

        self.get_logger().info(
            f"Butonlar: A={self.button_a_index}, "
            f"B={self.button_b_index}, "
            f"X={self.button_x_index}, "
            f"Y={self.button_y_index}"
        )
        self.get_logger().info(f"A komutu: {command_a}")
        self.get_logger().info(f"B komutu: {command_b}")
        self.get_logger().info(f"Y komutu: {command_y}")
        self.get_logger().info(f"Map save root: {self.map_save_root}")
        self.get_logger().info(f"Map save command: {self.map_save_command_str}")
        self.get_logger().info("/joy topic bekleniyor...")

    def check_readiness(self):
        publishers = self.count_publishers('/joy')

        if not self.joy_ready:
            if publishers > 0:
                self.get_logger().info(
                    f"/joy için publisher bulundu ({publishers} adet), ilk mesaj bekleniyor..."
                )
            else:
                self.get_logger().warning("/joy publisher henüz yok...")

    def joy_callback(self, msg: Joy):
        if not self.first_joy_received:
            self.first_joy_received = True
            self.joy_ready = True
            self.get_logger().info("/joy topic alındı, sistem hazır.")

            # İlk mesajdaki mevcut basılı tuşlar yanlış tetikleme yapmasın diye senkronla
            for key, index in self.button_map.items():
                if index < len(msg.buttons):
                    self.prev_states[key] = msg.buttons[index]
            return

        if not self.joy_ready:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9

        for key, index in self.button_map.items():
            if index >= len(msg.buttons):
                continue

            current = msg.buttons[index]
            previous = self.prev_states[key]

            if current == 1 and previous == 0:
                dt = now_sec - self.last_press_times[key]
                if dt >= self.debounce_sec:
                    self.get_logger().info(f"{key} tuşu tetiklendi.")
                    self.handle_button(key)
                    self.last_press_times[key] = now_sec

            self.prev_states[key] = current

    def handle_button(self, key: str):
        if key == 'A':
            self.launches['A'].toggle()
        elif key == 'B':
            self.launches['B'].toggle()
        elif key == 'X':
            self.save_map()
        elif key == 'Y':
            self.toggle_y()

    def toggle_y(self):
        launch_y = self.launches['Y']

        if not launch_y.command:
            self.get_logger().warning(
                "Y için komut tanımlı değil. 'command_y' parametresini doldur."
            )
            return

        launch_y.toggle()

    def save_map(self):
        self.get_logger().info("save_map() çağrıldı.")

        if not self.map_save_command_str:
            self.get_logger().error("Harita kayıt komutu boş.")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        target_dir = os.path.join(self.map_save_root, timestamp)
        map_base_path = os.path.join(target_dir, timestamp)

        try:
            os.makedirs(target_dir, exist_ok=True)
        except Exception as e:
            self.get_logger().error(
                f"Harita klasörü oluşturulamadı: {target_dir}, hata: {e}"
            )
            return

        command = shlex.split(self.map_save_command_str) + ['-f', map_base_path]

        self.get_logger().info(f"Harita kaydı başlatılıyor: {' '.join(command)}")
        self.get_logger().info(f"Kayıt klasörü: {target_dir}")

        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=30,
                check=False
            )

            self.get_logger().info(f"map_saver returncode: {result.returncode}")

            if result.stdout:
                self.get_logger().info(f"map_saver stdout: {result.stdout.strip()}")

            if result.stderr:
                self.get_logger().warning(f"map_saver stderr: {result.stderr.strip()}")

            if result.returncode == 0:
                self.get_logger().info(
                    f"Harita başarıyla kaydedildi: "
                    f"{map_base_path}.yaml ve {map_base_path}.pgm"
                )
            else:
                self.get_logger().error(
                    f"Harita kaydı başarısız. returncode={result.returncode}"
                )
        except subprocess.TimeoutExpired:
            self.get_logger().error("Harita kaydetme zaman aşımına uğradı.")
        except Exception as e:
            self.get_logger().error(f"Harita kaydetme hatası: {e}")

    def shutdown_launches(self):
        self.get_logger().info("Node kapanırken aktif launch grupları kapatılıyor...")
        for key, launch_obj in self.launches.items():
            if launch_obj.is_running():
                self.get_logger().info(f"{key} launch kapatılıyor...")
                launch_obj.stop()

    def destroy_node(self):
        self.shutdown_launches()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoyMultiToggle()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()