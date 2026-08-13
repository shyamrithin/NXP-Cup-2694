import sys
import rclpy
from rclpy.node import Node
from synapse_msgs.msg import ServerCommunication

# Default mission pairings embedded directly in the script
DEFAULT_PAIRINGS = [
    ("A", "X"),
    ("X", "B"),
    ("B", "Y"),
    ("Y", "C"),
    ("C", "Z"),
    ("Z", "OK"),
    ("PARKED","OK")
]


class ServerCommNode(Node):
    def __init__(self):
        super().__init__('server_comm_node')

        #self.team_name = team_name
        #self.team_try_count = team_try_count

        # Mission Pairings
        self.pairings = DEFAULT_PAIRINGS.copy()

        # Communication Protocol State
        self.server_uid = 50
        self.retry_count = 0
        self.pending_msg = None
        self.pending_uid = None
        self.last_processed_buggy_uid = -1

        self.retry_timer = None

        # ROS 2 Subscription (ONLY /ServerCommunication)
        self.sub_server_comm = self.create_subscription(
            ServerCommunication, '/ServerCommunication', self.server_comm_callback, 10
        )

        # ROS 2 Publisher
        self.pub_server_comm = self.create_publisher(
            ServerCommunication, '/ServerCommunication', 10
        )

        self.get_logger().info('Standalone Server Comm Node running. Listening exclusively to /ServerCommunication.')

    def server_comm_callback(self, message: ServerCommunication):
        # Filter messages destined for Server (dest=2)
        if message.dest != 2:
            return

        clean_msg = message.msg.strip().upper()

        # ── CASE 1: Buggy ACKed server's message (ack=1) ──
        if message.ack == 1:
            if message.uid == self.pending_uid:
                self.get_logger().info(f"[SUCCESS] Buggy ACKed server response (uid={message.uid}). Message delivered successfully!")
                self.pending_msg = None
                self.pending_uid = None
                self.retry_count = 0
                if self.retry_timer is not None:
                    self.retry_timer.cancel()
                    self.retry_timer = None
            else:
                self.get_logger().info(f"[RX ACK] Received ACK from Buggy for uid={message.uid}")
            return

        # ── CASE 2: New location request from Buggy (ack=0) ──
        if message.ack == 0:
            self.get_logger().info(f"[RX REQUEST] Received request from Buggy: '{clean_msg}' (uid={message.uid})")

            # 1. Send immediate ACK back to Buggy
            self._send_immediate_ack(message.uid)

            # Avoid duplicate processing for repeated request UIDs
            if self.last_processed_buggy_uid == message.uid:
                return
            self.last_processed_buggy_uid = message.uid

            # 2. Determine next target location
            next_target = self._get_next_target(clean_msg)

            self.server_uid += 1
            out_msg = ServerCommunication()
            out_msg.src = 2
            out_msg.dest = 1
            out_msg.uid = self.server_uid
            out_msg.ack = 0
            out_msg.msg = next_target

            self.pending_msg = next_target
            self.pending_uid = self.server_uid
            self.retry_count = 0

            self.pub_server_comm.publish(out_msg)
            self.get_logger().info(f"[TX TARGET] Sent Next Target '{next_target}' (uid={self.server_uid}) to Buggy")

            # Arm retry timer in case Buggy drops the message
            if self.retry_timer is not None:
                self.retry_timer.cancel()
            self.retry_timer = self.create_timer(1.0, self._retry_target_send)

    def _send_immediate_ack(self, uid):
        ack_msg = ServerCommunication()
        ack_msg.src = 2
        ack_msg.dest = 1
        ack_msg.uid = uid
        ack_msg.ack = 1
        ack_msg.msg = ""
        self.pub_server_comm.publish(ack_msg)
        self.get_logger().info(f"[TX ACK] Sent immediate ACK for uid={uid} to Buggy")

    def _get_next_target(self, current_loc: str) -> str:
        clean_curr = current_loc.strip().upper()
        for src, dest in self.pairings:
            if src == clean_curr:
                return dest
        fallback_map = {"A": "X", "X": "B", "B": "Y", "Y": "C", "C": "Z", "Z": "OK", "PARKED": "OK"}
        return fallback_map.get(clean_curr, "INVALID")

    def _retry_target_send(self):
        if self.pending_msg is None:
            if self.retry_timer is not None:
                self.retry_timer.cancel()
                self.retry_timer = None
            return

        self.retry_count += 1
        if self.retry_count > 5:
            self.get_logger().error(f"[RETRY FAILED] No ACK from Buggy after 5 retries for uid={self.pending_uid}.")
            if self.retry_timer is not None:
                self.retry_timer.cancel()
                self.retry_timer = None
            return

        out_msg = ServerCommunication()
        out_msg.src = 2
        out_msg.dest = 1
        out_msg.uid = self.pending_uid
        out_msg.ack = 0
        out_msg.msg = self.pending_msg
        self.pub_server_comm.publish(out_msg)
        self.get_logger().info(f"[TX RETRY] Retry {self.retry_count}/5 sending '{self.pending_msg}' (uid={self.pending_uid})")


def main():
    rclpy.init()
    server_comm_node = ServerCommNode()

    try:
        rclpy.spin(server_comm_node)
    except KeyboardInterrupt:
        print("\n[Server Comm Node] Shutting down...")
    finally:
        server_comm_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
