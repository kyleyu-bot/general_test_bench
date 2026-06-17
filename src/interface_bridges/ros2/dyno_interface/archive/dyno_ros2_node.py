#!/usr/bin/env python3
"""
Dyno ROS2 Bridge Node

Receives telemetry JSON from bridge_udp (UDP port 7600) and publishes it
as ROS2 topics.  Subscribes to command topics and forwards them back to
bridge_udp (UDP port 7601).

Topics published:
  /dyno/main_drive/status   (std_msgs/String — JSON)
  /dyno/dut/status          (std_msgs/String — JSON)
  /dyno/encoder/count       (std_msgs/UInt32)
  /dyno/torque/ch1          (std_msgs/Float64)
  /dyno/torque/ch2          (std_msgs/Float64)
  /dyno/loop/stats          (std_msgs/String — JSON with cycle/wkc/cycle_us)

Topics subscribed:
  /dyno/command             (std_msgs/String — JSON matching ipc_types Command)

Usage:
  ros2 run dyno_ros2_bridge dyno_ros2_node [--ros-args -p telem_port:=7600]
"""

import json
import socket
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String, UInt32

TELEM_PORT  = 7600
COMMAND_PORT = 7601
BUFFER_SIZE  = 4096


class DynoRos2Node(Node):

    def __init__(self):
        super().__init__('dyno_ros2_bridge')

        # Parameters (overridable via --ros-args -p key:=value)
        self.declare_parameter('telem_port',   TELEM_PORT)
        self.declare_parameter('cmd_port',     COMMAND_PORT)
        self.declare_parameter('bridge_host',  '127.0.0.1')

        telem_port  = self.get_parameter('telem_port').value
        self.cmd_port    = self.get_parameter('cmd_port').value
        self.bridge_host = self.get_parameter('bridge_host').value

        # Publishers
        self.pub_main   = self.create_publisher(String,  '/dyno/main_drive/status', 10)
        self.pub_dut    = self.create_publisher(String,  '/dyno/dut/status',         10)
        self.pub_enc    = self.create_publisher(UInt32,  '/dyno/encoder/count',      10)
        self.pub_ch1_t  = self.create_publisher(Float64, '/dyno/torque/ch1',         10)
        self.pub_ch2_t  = self.create_publisher(Float64, '/dyno/torque/ch2',         10)
        self.pub_stats  = self.create_publisher(String,  '/dyno/loop/stats',         10)

        # Subscriber — commands from ROS2 → bridge_udp
        self.sub_cmd = self.create_subscription(
            String, '/dyno/command', self._on_command, 10)

        # UDP sockets
        self._telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._telem_sock.bind(('127.0.0.1', telem_port))
        self._telem_sock.settimeout(1.0)

        self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Receive thread
        self._running = True
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

        self.get_logger().info(
            f'DynoRos2Node started | '
            f'telem=:{telem_port}  cmd={self.bridge_host}:{self.cmd_port}'
        )

    def _recv_loop(self):
        while self._running:
            try:
                data, _ = self._telem_sock.recvfrom(BUFFER_SIZE)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'Recv error: {e}')
                continue

            try:
                t = json.loads(data.decode())
            except json.JSONDecodeError:
                continue

            self._publish(t)

    def _publish(self, t: dict):
        # Loop stats
        stats_msg = String()
        stats_msg.data = json.dumps({
            'cycle':    t.get('cycle', 0),
            'wkc':      t.get('wkc', 0),
            'cycle_us': t.get('t_us', 0.0),
        })
        self.pub_stats.publish(stats_msg)

        # Drive status
        main_msg = String()
        main_msg.data = json.dumps(t.get('main', {}))
        self.pub_main.publish(main_msg)

        dut_msg = String()
        dut_msg.data = json.dumps(t.get('dut', {}))
        self.pub_dut.publish(dut_msg)

        # Encoder
        enc_msg = UInt32()
        enc_msg.data = int(t.get('enc', 0))
        self.pub_enc.publish(enc_msg)

        # Torque
        ch1_msg = Float64()
        ch1_msg.data = float(t.get('ch1_t', 0.0))
        self.pub_ch1_t.publish(ch1_msg)

        ch2_msg = Float64()
        ch2_msg.data = float(t.get('ch2_t', 0.0))
        self.pub_ch2_t.publish(ch2_msg)

    def _on_command(self, msg: String):
        """Forward a JSON command string from ROS2 topic → bridge_udp."""
        try:
            # Validate it parses cleanly before forwarding.
            json.loads(msg.data)
            self._cmd_sock.sendto(
                msg.data.encode(),
                (self.bridge_host, self.cmd_port)
            )
        except Exception as e:
            self.get_logger().warn(f'Command forward failed: {e}')

    def destroy_node(self):
        self._running = False
        self._recv_thread.join(timeout=2.0)
        self._telem_sock.close()
        self._cmd_sock.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = DynoRos2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
