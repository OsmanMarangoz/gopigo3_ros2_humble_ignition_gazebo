import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time

LINEAR_SPEED = 0.15   # Geschwindigkeit für Vorwärtsbewegung
ANGULAR_SPEED = 0.4  # Geschwindigkeit für Drehungen
PATH_RESOLUTION = 0.01 # Speichere jeden cm einen Punkt

class pfadkreuzung(Node):
    def __init__(self):
        super().__init__('pfadkreuzung')
        self.cmd_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_points = []
        self.get_logger().info("Einfacher Pfad-Runner gestartet.")

    def odom_callback(self, msg: Odometry):
        current_pos = msg.pose.pose.position
        if not self.path_points:
            self.path_points.append((current_pos.x, current_pos.y))
            return
        last_point = self.path_points[-1]
        dist_from_last = math.sqrt((current_pos.x - last_point[0])**2 + (current_pos.y - last_point[1])**2)
        if dist_from_last > PATH_RESOLUTION:
            self.path_points.append((current_pos.x, current_pos.y))

    def move_forward(self, duration):
        """Bewegt den Roboter für eine bestimmte Dauer vorwärts."""
        self.get_logger().info(f"Fahre {duration:.1f}s vorwärts...")
        twist = Twist()
        twist.linear.x = LINEAR_SPEED
        self.cmd_pub.publish(twist)
        start_time = time.time()

        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.stop_robot()
        rclpy.spin_once(self, timeout_sec=0.1)


    def turn(self, duration):
        """Dreht den Roboter für eine bestimmte Dauer."""
        self.get_logger().info(f"Drehe für {duration:.1f}s...")
        twist = Twist()
        twist.angular.z = -ANGULAR_SPEED
        self.cmd_pub.publish(twist)
        start_time = time.time()

        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.stop_robot()
        rclpy.spin_once(self, timeout_sec=0.1)


    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def check_path_crossing(self):
        """Prüft am Ende, ob ein Punkt doppelt besucht wurde."""
        self.get_logger().info("Prüfe den Pfad auf Kreuzungen...")
        for i in range(len(self.path_points)):
            for j in range(i + 20, len(self.path_points)):
                p1 = self.path_points[i]
                p2 = self.path_points[j]
                distance = math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
                if distance < PATH_RESOLUTION:
                    self.get_logger().warn(f"KREUZUNG GEFUNDEN bei Punkt {p1} und {p2}")
                    return True
        self.get_logger().info("Keine Kreuzung gefunden.")
        return False

def main(args=None):
    rclpy.init(args=args)
    node = pfadkreuzung()
    time.sleep(2) 
    node.move_forward(duration=5.0)
    node.turn(duration=4.35) 
    node.move_forward(duration=5.0)
    node.turn(duration=4.35) 
    node.move_forward(duration=5.0)
    node.turn(duration=4.35) 
    node.move_forward(duration=4.0)
    node.turn(duration=4.35) 
    node.move_forward(duration=4.0)
    node.turn(duration=4.35) 
    node.move_forward(duration=5.0)
    node.check_path_crossing()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()