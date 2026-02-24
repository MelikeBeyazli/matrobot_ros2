import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Header

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Quaternion

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

        self.declare_parameter('wheel_radius', 0.075)  # meters
        self.declare_parameter('wheel_base', 0.41)     # meters

        self.declare_parameter('odom_topic', '/odometry_wheel')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('joint_states_topic', '/joint_states')

        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('imu_frame_id', 'imu_link')

        # TF yayınlama (EKF varsa genelde false)
        self.declare_parameter('publish_tf', False)

        # Arduino IMU kullan / kullanma
        self.declare_parameter('use_imu', False)

        # cmd gönderme
        self.declare_parameter('cmd_rate_hz', 50.0)
        self.declare_parameter('cmd_timeout_s', 0.4)

        # cmd clamp
        self.declare_parameter('clamp_cmd', True)
        self.declare_parameter('max_v', 1.2)
        self.declare_parameter('max_w', 3.0)

        # Odometry covariance (basit)
        self.declare_parameter('odom_cov_x', 0.05)
        self.declare_parameter('odom_cov_y', 0.05)
        self.declare_parameter('odom_cov_yaw', 0.10)

        # Serial read settings
        self.declare_parameter('serial_read_hz', 200.0)   # line parsing loop
        self.declare_parameter('serial_timeout_s', 0.01)  # pyserial timeout

        # -------------------------
        # Pull params
        # -------------------------
        self.port = self.get_parameter('serial_port').value   # FIX: port -> serial_port
        self.baudrate = int(self.get_parameter('baudrate').value)

        self.left_joint_name = self.get_parameter('left_joint_name').value
        self.right_joint_name = self.get_parameter('right_joint_name').value

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value

        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value

        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.use_arduino_imu = bool(self.get_parameter('use_imu').value)

        self.cmd_rate_hz = float(self.get_parameter('cmd_rate_hz').value)
        self.cmd_timeout_s = float(self.get_parameter('cmd_timeout_s').value)

        self.clamp_cmd = bool(self.get_parameter('clamp_cmd').value)
        self.max_v = float(self.get_parameter('max_v').value)
        self.max_w = float(self.get_parameter('max_w').value)

        self.odom_cov_x = float(self.get_parameter('odom_cov_x').value)
        self.odom_cov_y = float(self.get_parameter('odom_cov_y').value)
        self.odom_cov_yaw = float(self.get_parameter('odom_cov_yaw').value)

        self.serial_read_hz = float(self.get_parameter('serial_read_hz').value)
        self.serial_timeout_s = float(self.get_parameter('serial_timeout_s').value)

        # -------------------------
        # Publishers/Subscribers
        # -------------------------
        self.pub_js = self.create_publisher(JointState, self.joint_states_topic, 10)
        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, 10)

        # IMU publisher sadece gerekiyorsa
        self.pub_imu = None
        if self.use_arduino_imu:
            self.pub_imu = self.create_publisher(Imu, self.imu_topic, 50)
            self.get_logger().info("Arduino IMU ENABLED: publishing IMU from serial.")
        else:
            self.get_logger().info("Arduino IMU DISABLED: ignoring IMU lines from serial.")

        self.sub_cmd = self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # -------------------------
        # State
        # -------------------------
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.have_last_wheel_pos = False
        self.last_left_pos = 0.0
        self.last_right_pos = 0.0
        self.last_odom_time = self.get_clock().now()

        self.rx_buffer = ""

        # -------------------------
        # Serial open
        # -------------------------
        self.ser: Optional[serial.Serial] = None
        self._open_serial()

        # Timers
        self.cmd_timer = self.create_timer(1.0 / max(self.cmd_rate_hz, 1.0), self._cmd_timer_cb)
        self.read_timer = self.create_timer(1.0 / max(self.serial_read_hz, 10.0), self._read_timer_cb)

        self.get_logger().info(
            f"[HardwareNode] Ready port={self.port} baud={self.baudrate} publish_tf={self.publish_tf} use_arduino_imu={self.use_arduino_imu}"
        )

    def _open_serial(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.serial_timeout_s
            )
        except Exception as e:
            self.get_logger().error(f"Failed to open serial {self.port}: {e}")
            raise

    def _on_cmd_vel(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        if self.clamp_cmd:
            v = max(-self.max_v, min(self.max_v, v))
            w = max(-self.max_w, min(self.max_w, w))

        self.v_cmd = v
        self.w_cmd = w
        self.last_cmd_time = self.get_clock().now()

    def _cmd_timer_cb(self):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds * 1e-9

        v = self.v_cmd
        w = self.w_cmd
        if dt > self.cmd_timeout_s:
            v = 0.0
            w = 0.0

        line = f"CMD {v:.4f} {w:.4f}\n"
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(line.encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")

    def _read_timer_cb(self):
        if not self.ser or not self.ser.is_open:
            return

        try:
            data = self.ser.read(512)
            if not data:
                return
            self.rx_buffer += data.decode('utf-8', errors='ignore')

            while '\n' in self.rx_buffer:
                line, self.rx_buffer = self.rx_buffer.split('\n', 1)
                line = line.strip('\r').strip()
                if line:
                    self._handle_line(line)

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

    def _handle_line(self, line: str):
        parts = line.split()
        if not parts:
            return

        if parts[0] == "JS" and len(parts) >= 9:
            self._parse_js(parts)

        elif parts[0] == "IMU" and len(parts) >= 15:
            # IMU sadece param true ise
            if self.use_arduino_imu and self.pub_imu is not None:
                self._parse_imu(parts)
            # değilse tamamen ignore

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

    def _parse_imu(self, p):
        # IMU seq t_us dt_us gx gy gz ax ay az mx my mz status mag_type
        try:
            gx = float(p[4]); gy = float(p[5]); gz = float(p[6])
            ax = float(p[7]); ay = float(p[8]); az = float(p[9])
        except Exception:
            return

        stamp = self.get_clock().now().to_msg()

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.imu_frame_id

        imu.orientation_covariance[0] = -1.0  # orientation unknown

        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz

        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az

        imu.angular_velocity_covariance[0] = 0.02
        imu.angular_velocity_covariance[4] = 0.02
        imu.angular_velocity_covariance[8] = 0.02

        imu.linear_acceleration_covariance[0] = 0.2
        imu.linear_acceleration_covariance[4] = 0.2
        imu.linear_acceleration_covariance[8] = 0.2

        if self.pub_imu is not None:
            self.pub_imu.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.ser and node.ser.is_open:
                node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()