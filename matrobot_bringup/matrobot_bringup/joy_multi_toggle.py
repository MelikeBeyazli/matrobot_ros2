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


class ManagedProcess:
    def __init__(self, name: str, command: List[str], logger):
        self.name = name
        self.command = command
        self.logger = logger
        self.process: Optional[subprocess.Popen] = None
        self.pgid: Optional[int] = None

    def is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def start(self):
        if not self.command:
            self.logger.error(f"{self.name} için komut boş.")
            return

        if self.is_running():
            self.logger.info(f"{self.name} zaten çalışıyor.")
            return

        try:
            self.logger.info(f"{self.name} başlatılıyor: {' '.join(self.command)}")
            self.process = subprocess.Popen(
                self.command,
                preexec_fn=os.setsid
            )
            self.pgid = os.getpgid(self.process.pid)
            self.logger.info(f"{self.name} başladı. PID={self.process.pid}")
        except Exception as e:
            self.logger.error(f"{self.name} başlatılamadı: {e}")
            self.process = None
            self.pgid = None

    def stop(self):
        if self.process is None:
            return

        try:
            if self.process.poll() is None:
                self.logger.info(f"{self.name} durduruluyor...")
                if self.pgid is not None:
                    os.killpg(self.pgid, signal.SIGINT)
                else:
                    self.process.send_signal(signal.SIGINT)

                self.process.wait(timeout=5.0)
                self.logger.info(f"{self.name} düzgün şekilde durdu.")
            else:
                self.logger.info(f"{self.name} zaten kapanmış.")
        except subprocess.TimeoutExpired:
            self.logger.warning(f"{self.name} SIGINT sonrası kapanmadı, SIGTERM deneniyor...")
            try:
                if self.pgid is not None:
                    os.killpg(self.pgid, signal.SIGTERM)
                else:
                    self.process.terminate()
                self.process.wait(timeout=3.0)
            except Exception as e:
                self.logger.warning(f"{self.name} SIGTERM başarısız: {e}, SIGKILL uygulanıyor...")
                try:
                    if self.pgid is not None:
                        os.killpg(self.pgid, signal.SIGKILL)
                    else:
                        self.process.kill()
                except Exception as kill_e:
                    self.logger.error(f"{self.name} kill başarısız: {kill_e}")
        except Exception as e:
            self.logger.error(f"{self.name} durdurma hatası: {e}")
        finally:
            self.process = None
            self.pgid = None

    def toggle(self):
        if self.is_running():
            self.stop()
        else:
            self.start()


class JoyMultiToggle(Node):
    def __init__(self):
        super().__init__('joy_multi_toggle')

        self.declare_parameter('button_a_index', 0)
        self.declare_parameter('button_b_index', 1)
        self.declare_parameter('button_x_index', 2)
        self.declare_parameter('button_y_index', 3)
        self.declare_parameter('debounce_sec', 0.35)

        self.declare_parameter('command_a', '')
        self.declare_parameter('command_b', '')
        self.declare_parameter('command_x', '')

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

        command_a_str = str(self.get_parameter('command_a').value)
        command_b_str = str(self.get_parameter('command_b').value)
        command_x_str = str(self.get_parameter('command_x').value)

        self.map_save_root = os.path.expanduser(
            str(self.get_parameter('map_save_root').value)
        )
        self.map_save_command_str = str(self.get_parameter('map_save_command').value).strip()

        command_a = shlex.split(command_a_str) if command_a_str else []
        command_b = shlex.split(command_b_str) if command_b_str else []
        command_x = shlex.split(command_x_str) if command_x_str else []

        self.processes: Dict[str, ManagedProcess] = {
            'A': ManagedProcess('bringup', command_a, self.get_logger()),
            'B': ManagedProcess('cmd_vel_teleop', command_b, self.get_logger()),
            'X': ManagedProcess('slam', command_x, self.get_logger()),
        }

        self.button_map = {
            'A': self.button_a_index,
            'B': self.button_b_index,
            'X': self.button_x_index,
            'Y': self.button_y_index,
        }

        self.prev_states = {'A': 0, 'B': 0, 'X': 0, 'Y': 0}
        self.last_toggle_times = {'A': 0.0, 'B': 0.0, 'X': 0.0, 'Y': 0.0}

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.get_logger().info(
            f"Buton map: A={self.button_a_index}, "
            f"B={self.button_b_index}, "
            f"X={self.button_x_index}, "
            f"Y={self.button_y_index}"
        )
        self.get_logger().info(f"A komutu: {command_a}")
        self.get_logger().info(f"B komutu: {command_b}")
        self.get_logger().info(f"X komutu: {command_x}")
        self.get_logger().info(f"Harita kayıt kökü: {self.map_save_root}")
        self.get_logger().info(f"Harita kayıt komutu: {self.map_save_command_str}")
        self.get_logger().info("Joy multi toggle hazır.")

    def joy_callback(self, msg: Joy):
        now_sec = self.get_clock().now().nanoseconds / 1e9

        for key, index in self.button_map.items():
            if index >= len(msg.buttons):
                continue

            current = msg.buttons[index]
            previous = self.prev_states[key]

            if current == 1 and previous == 0:
                dt = now_sec - self.last_toggle_times[key]
                if dt >= self.debounce_sec:
                    self.get_logger().info(f"{key} tuşu tetiklendi.")

                    if key == 'Y':
                        self.save_map()
                    else:
                        self.processes[key].toggle()

                    self.last_toggle_times[key] = now_sec

            self.prev_states[key] = current

    def save_map(self):
        slam_process = self.processes.get('X')

        if slam_process is None or not slam_process.is_running():
            self.get_logger().warning(
                "SLAM çalışmıyor. Önce X ile SLAM başlatılmalı, sonra Y ile harita kaydedilmeli."
            )
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

        if not self.map_save_command_str:
            self.get_logger().error("Harita kayıt komutu boş.")
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

    def destroy_node(self):
        for proc in self.processes.values():
            if proc.is_running():
                proc.stop()
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