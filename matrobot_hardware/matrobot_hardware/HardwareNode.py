#!/usr/bin/env python3
import math
import threading
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from tf2_ros import TransformBroadcaster

import serial


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    half = 0.5 * yaw
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


class HardwareNode(Node):
    def __init__(self):
        super().__init__('hardware_node')

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        self.declare_parameter('left_joint_name', 'left_wheel_joint')
        self.declare_parameter('right_joint_name', 'right_wheel_joint')

        self.declare_parameter('wheel_radius', 0.09)   # meters
        self.declare_parameter('wheel_base', 0.435)    # meters

        self.declare_parameter('odom_topic', '/odometry/wheel')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('joint_states_topic', '/joint_states')

        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')

        self.declare_parameter('publish_tf', False)

        # cmd clamp
        self.declare_parameter('clamp_cmd', True)
        self.declare_parameter('max_v', 0.6)
        self.declare_parameter('max_w', 2.6)

        # Open-loop v,w -> PWM mapping (TUNE)
        self.declare_parameter('v_to_pwm', 220.0)
        self.declare_parameter('w_to_pwm', 120.0)
        self.declare_parameter('pwm_max', 255)

        # cmd timeout
        self.declare_parameter('cmd_timeout_s', 0.4)

        # Odometry covariance (basit)
        self.declare_parameter('odom_cov_x', 0.05)
        self.declare_parameter('odom_cov_y', 0.05)
        self.declare_parameter('odom_cov_yaw', 0.10)

        # Serial settings
        self.declare_parameter('serial_timeout_s', 0.002)     # RX read timeout (small)
        self.declare_parameter('serial_write_timeout_s', 0.0) # non-blocking write

        # Thread rates
        self.declare_parameter('tx_rate_hz', 200.0)  # PWM send rate
        self.declare_parameter('rx_chunk_bytes', 512)

        # -------------------------
        # Pull params
        # -------------------------
        self.port = str(self.get_parameter('serial_port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)

        self.left_joint_name = str(self.get_parameter('left_joint_name').value)
        self.right_joint_name = str(self.get_parameter('right_joint_name').value)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.joint_states_topic = str(self.get_parameter('joint_states_topic').value)

        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)

        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.clamp_cmd = bool(self.get_parameter('clamp_cmd').value)
        self.max_v = float(self.get_parameter('max_v').value)
        self.max_w = float(self.get_parameter('max_w').value)

        self.v_to_pwm = float(self.get_parameter('v_to_pwm').value)
        self.w_to_pwm = float(self.get_parameter('w_to_pwm').value)
        self.pwm_max = int(self.get_parameter('pwm_max').value)

        self.cmd_timeout_s = float(self.get_parameter('cmd_timeout_s').value)

        self.odom_cov_x = float(self.get_parameter('odom_cov_x').value)
        self.odom_cov_y = float(self.get_parameter('odom_cov_y').value)
        self.odom_cov_yaw = float(self.get_parameter('odom_cov_yaw').value)

        self.serial_timeout_s = float(self.get_parameter('serial_timeout_s').value)
        self.serial_write_timeout_s = float(self.get_parameter('serial_write_timeout_s').value)

        self.tx_rate_hz = float(self.get_parameter('tx_rate_hz').value)
        self.rx_chunk_bytes = int(self.get_parameter('rx_chunk_bytes').value)

        # -------------------------
        # Publishers/Subscribers
        # -------------------------
        self.pub_js = self.create_publisher(JointState, self.joint_states_topic, 10)
        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, 10)

        self.sub_cmd = self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # -------------------------
        # State
        # -------------------------
        self._cmd_lock = threading.Lock()
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.last_cmd_mono = time.monotonic()

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.have_last_wheel_pos = False
        self.last_left_pos = 0.0
        self.last_right_pos = 0.0
        self.last_odom_time = self.get_clock().now()

        self.rx_buffer = ""

        # Serial
        self.ser: Optional[serial.Serial] = None

        # Threads stop event (serial connect loop bunu kullanacak)
        self._stop_evt = threading.Event()

        # İlk bağlantı: port açılana kadar dene
        self._ensure_serial_connected()

        # Threads
        self._tx_thread = threading.Thread(target=self._tx_loop, name="serial_tx", daemon=True)
        self._rx_thread = threading.Thread(target=self._rx_loop, name="serial_rx", daemon=True)
        self._tx_thread.start()
        self._rx_thread.start()

    # -------------------------
    # Serial connect helpers
    # -------------------------
    def _close_serial(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    def _try_open_serial_once(self) -> bool:
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.serial_timeout_s,
                write_timeout=self.serial_write_timeout_s,
            )
            try:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except Exception:
                pass

            self.get_logger().info(f"Serial opened: {self.port} @ {self.baudrate}")
            return True
        except Exception as e:
            self._close_serial()
            self.get_logger().warn(f"Serial not available ({self.port}): {e}")
            return False

    def _ensure_serial_connected(self):
        """
        Port açılana kadar dener. Node kapatılırken döngü biter.
        """
        retry_interval_s = 1.0
        while rclpy.ok() and not self._stop_evt.is_set():
            if self._try_open_serial_once():
                return
            time.sleep(retry_interval_s)

    # -------------------------
    # cmd_vel callback
    # -------------------------
    def _on_cmd_vel(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        if self.clamp_cmd:
            v = max(-self.max_v, min(self.max_v, v))
            w = max(-self.max_w, min(self.max_w, w))

        with self._cmd_lock:
            self.v_cmd = v
            self.w_cmd = w
            self.last_cmd_mono = time.monotonic()

    def _vw_to_pwm(self, v: float, w: float) -> Tuple[int, int]:
        base = v * self.v_to_pwm
        turn = w * self.w_to_pwm

        pwm_l = int(round(base - turn))
        pwm_r = int(round(base + turn))

        pwm_l = max(-self.pwm_max, min(self.pwm_max, pwm_l))
        pwm_r = max(-self.pwm_max, min(self.pwm_max, pwm_r))
        return pwm_l, pwm_r

    # -------------------------
    # TX loop: sürekli PWM gönder
    # -------------------------
    def _tx_loop(self):
        period = 1.0 / max(self.tx_rate_hz, 1.0)
        next_t = time.monotonic()

        while not self._stop_evt.is_set():
            # Serial yoksa (kablo çıkmışsa vs) tekrar bağlan
            if not self.ser or not self.ser.is_open:
                self._ensure_serial_connected()
                # bağlanamadıysa stop/shutdown olabilir
                time.sleep(0.01)
                continue

            now = time.monotonic()

            # drift azaltmak için next_t kovalayalım
            if now < next_t:
                time.sleep(next_t - now)
                continue

            next_t += period
            if next_t < now - 5.0 * period:
                next_t = now + period

            with self._cmd_lock:
                v = self.v_cmd
                w = self.w_cmd
                age = now - self.last_cmd_mono

            if age > self.cmd_timeout_s:
                v = 0.0
                w = 0.0

            pwm_l, pwm_r = self._vw_to_pwm(v, w)
            line = f"PWM {pwm_l} {pwm_r}\n"

            try:
                self.ser.write(line.encode("utf-8"))
            except Exception as e:
                self.get_logger().warn(f"Serial TX error: {e} (will reconnect)")
                self._close_serial()
                time.sleep(0.05)

    # -------------------------
    # RX loop: sürekli oku ve parse et
    # -------------------------
    def _rx_loop(self):
        while not self._stop_evt.is_set():
            # Serial yoksa tekrar bağlan
            if not self.ser or not self.ser.is_open:
                self._ensure_serial_connected()
                time.sleep(0.01)
                continue

            try:
                data = self.ser.read(self.rx_chunk_bytes)
                if not data:
                    continue

                self.rx_buffer += data.decode("utf-8", errors="ignore")

                while "\n" in self.rx_buffer:
                    line, self.rx_buffer = self.rx_buffer.split("\n", 1)
                    line = line.strip("\r").strip()
                    if line:
                        self._handle_line(line)

            except Exception as e:
                self.get_logger().warn(f"Serial RX error: {e} (will reconnect)")
                self._close_serial()
                time.sleep(0.05)

    def _handle_line(self, line: str):
        parts = line.split()
        if not parts:
            return

        if parts[0] == "JS" and len(parts) >= 9:
            self._parse_js(parts)

    def _parse_js(self, p):
        # JS seq t_us Lpos Rpos Lvel Rvel Lticks Rticks
        try:
            lpos = float(p[3])
            rpos = float(p[4])
            lvel = float(p[5])
            rvel = float(p[6])
        except Exception:
            return

        stamp = self.get_clock().now().to_msg()

        # JointState
        js = JointState()
        js.header = Header()
        js.header.stamp = stamp
        js.name = [self.left_joint_name, self.right_joint_name]
        js.position = [lpos, rpos]
        js.velocity = [lvel, rvel]
        self.pub_js.publish(js)

        # Odometry integration from wheel positions
        now = self.get_clock().now()
        if not self.have_last_wheel_pos:
            self.have_last_wheel_pos = True
            self.last_left_pos = lpos
            self.last_right_pos = rpos
            self.last_odom_time = now
            return

        dt = (now - self.last_odom_time).nanoseconds * 1e-9
        if dt <= 1e-6:
            return
        self.last_odom_time = now

        dL = (lpos - self.last_left_pos) * self.wheel_radius
        dR = (rpos - self.last_right_pos) * self.wheel_radius
        self.last_left_pos = lpos
        self.last_right_pos = rpos

        ds = 0.5 * (dR + dL)
        dyaw = (dR - dL) / max(self.wheel_base, 1e-9)

        yaw_mid = self.yaw + 0.5 * dyaw
        self.x += ds * math.cos(yaw_mid)
        self.y += ds * math.sin(yaw_mid)
        self.yaw += dyaw

        v = self.wheel_radius * 0.5 * (rvel + lvel)
        w = self.wheel_radius * (rvel - lvel) / max(self.wheel_base, 1e-9)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quat(self.yaw)

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = self.odom_cov_x
        odom.pose.covariance[7] = self.odom_cov_y
        odom.pose.covariance[35] = self.odom_cov_yaw

        self.pub_odom.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        # threadleri durdur
        self._stop_evt.set()

        try:
            if self._tx_thread.is_alive():
                self._tx_thread.join(timeout=0.5)
            if self._rx_thread.is_alive():
                self._rx_thread.join(timeout=0.5)
        except Exception:
            pass

        self._close_serial()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
