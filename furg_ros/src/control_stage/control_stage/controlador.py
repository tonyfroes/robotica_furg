import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.subscription_laser = self.create_subscription(
            LaserScan,
            '/base_scan',
            self.laser_callback,
            10)
        
        # Publicador para o tópico de comando de velocidade
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Waypoints para navegação do robô
        self.waypoints: list[tuple[float, float]] = [(9.6, -6.7), (16.3, -1.1)]
        
        # Dados de laser e odometria inicializados como None
        self.laser_data: LaserScan = None
        self.odom_data: Odometry = None
        
        # Distância mínima para obstáculos
        self.min_distance_to_obstacle: float = 0.3
        
        # Timer para chamar a função de controle periodicamente
        self.timer = self.create_timer(0.1, self.control)
        
        # Variáveis de posição e orientação do robô
        self.position_robot_x: float = 0.0
        self.position_robot_y: float = 0.0
        self.orientation_robot_yaw: float = 0.0
        
        # Índice do waypoint atual e sinalizador de conclusão dos waypoints
        self.current_waypoint_index: int = 0
        self.all_waypoints_visited: bool = False
        
    def get_yaw_from_orientation(self, orientation: Odometry) -> float:
        """Calcula o ângulo yaw a partir da orientação quaternionica."""
        siny_cosp = 2 * (orientation.pose.pose.orientation.w * orientation.pose.pose.orientation.z +
                         orientation.pose.pose.orientation.x * orientation.pose.pose.orientation.y)
        cosy_cosp = 1 - 2 * (orientation.pose.pose.orientation.y ** 2 + orientation.pose.pose.orientation.z ** 2)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg: Odometry) -> None:
        """Callback para dados de odometria."""
        self.odom_data = msg
        self.position_robot_x = msg.pose.pose.position.x
        self.position_robot_y = msg.pose.pose.position.y
        self.orientation_robot_yaw = self.get_yaw_from_orientation(msg)
        
    def laser_callback(self, msg: LaserScan) -> None:
        """Callback para dados do laser scan."""
        self.laser_data = msg
        
    def normalize_angle(self, angle: float) -> float:
        """Normaliza o ângulo para o intervalo [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def control(self) -> None:
        """Função de controle do robô."""
        if self.laser_data is None or self.odom_data is None or self.all_waypoints_visited:
            return

        if isinstance(self.laser_data, LaserScan):
            min_distance_ahead = min(self.laser_data.ranges[50:155])
        else:
            self.get_logger().warning('Dados do laser não são do tipo LaserScan.')
            return
        
        cmd_vel = Twist()
        
        distance_to_waypoint = math.sqrt(
            (self.position_robot_x - self.waypoints[self.current_waypoint_index][0])**2 + 
            (self.position_robot_y - self.waypoints[self.current_waypoint_index][1])**2
        )
        
        goal_angle = math.atan2(
            self.waypoints[self.current_waypoint_index][1] - self.position_robot_y, 
            self.waypoints[self.current_waypoint_index][0] - self.position_robot_x
        )

        if distance_to_waypoint < 0.5:
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info('Deu bom!')
                self.all_waypoints_visited = True
                self.publisher_cmd_vel.publish(Twist())
                return
            
        """Controle de velocidade e direção do robô."""
        if min_distance_ahead < self.min_distance_to_obstacle:
            cmd_vel.angular.z = 0.5
            cmd_vel.linear.x = 0.0
        else:
            cmd_vel.linear.x = 0.5
            angle_diff = self.normalize_angle(goal_angle - self.orientation_robot_yaw)
            cmd_vel.angular.z = 1.0 * angle_diff

        self.publisher_cmd_vel.publish(cmd_vel)
                
def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
