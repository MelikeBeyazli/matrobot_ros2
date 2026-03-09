#!/usr/bin/env python3

import os
import shlex
import signal
import subprocess
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
        self.declare_parameter('debounce_sec', 0.35)

        # Komutları string olarak al, sonra split et
        self.declare_parameter('command_a', '')
        self.declare_parameter('command_b', '')
        self.declare_parameter('command_x', '')

        self.button_a_index = int(self.get_parameter('button_a_index').value)
        self.button_b_index = int(self.get_parameter('button_b_index').value)
        self.button_x_index = int(self.get_parameter('button_x_index').value)
        self.debounce_sec = float(self.get_parameter('debounce_sec').value)

        command_a_str = str(self.get_parameter('command_a').value)
        command_b_str = str(self.get_parameter('command_b').value)
        command_x_str = str(self.get_parameter('command_x').value)

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
        }

        self.prev_states = {'A': 0, 'B': 0, 'X': 0}
        self.last_toggle_times = {'A': 0.0, 'B': 0.0, 'X': 0.0}

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.get_logger().info(
            f"Buton map: A={self.button_a_index}, B={self.button_b_index}, X={self.button_x_index}"
        )
        self.get_logger().info(f"A komutu: {command_a}")
        self.get_logger().info(f"B komutu: {command_b}")
        self.get_logger().info(f"X komutu: {command_x}")
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
                    self.processes[key].toggle()
                    self.last_toggle_times[key] = now_sec

            self.prev_states[key] = current

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