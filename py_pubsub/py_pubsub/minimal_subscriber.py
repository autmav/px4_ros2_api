import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
import matplotlib.pyplot as plt

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.xs = []
        self.ys = []
        self.zs = []
        self.fig = plt.figure()
        self.ax = plt.axes(projection = '3d')
        self.setpoint_xs = [0, 0, 5, 5, 0]
        self.setpoint_ys = [0, 5, 5, 0, 0]
        self.setpoint_zs = [3, 3, 3, 3, 3]
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)

        self.vehicle_odometry = VehicleOdometry()

    def vehicle_odometry_callback(self, msg):
        """Callback function for msg topic subscriber."""
        self.vehicle_odometry = msg

        self.xs.append(self.vehicle_odometry.position[0])
        self.ys.append(self.vehicle_odometry.position[1])
        self.zs.append(-self.vehicle_odometry.position[2])
        plt.cla()
        self.ax.plot3D(self.xs, self.ys, self.zs)
        self.ax.plot3D(self.setpoint_xs, self.setpoint_ys, self.setpoint_zs)

        plt.pause(0.001)
      
        #print(self.vehicle_odometry.x)
    
        
    #    self.subscription  # prevent unused variable warning

#    def listener_callback(self, msg):
#        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
