import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import math
import csv
import numpy as np

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # === 설정 변수들 (튜닝 포인트) ===
        self.LOOKAHEAD_DISTANCE = 1.5  
        self.VELOCITY = 1.5           
        self.WHEELBASE = 0.33          
        self.CSV_FILENAME = 'clicked_waypoints.csv'

        # === 토픽 설정 ===
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)
        
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)

        # === 웨이포인트 로드 및 보간(Interpolation) ===
        self.waypoints = []
        self.load_and_interpolate_waypoints() # 함수 이름 변경
        
        self.get_logger().info("Pure Pursuit 노드 시작 (Interpolation Mode)")

    def load_and_interpolate_waypoints(self):
        """ 
        CSV를 읽은 뒤, 점과 점 사이를 촘촘하게 채워줍니다. (선형 보간)
        이러면 6m 간격으로 점을 찍어도 0.1m 간격처럼 부드럽게 주행합니다.
        """
        raw_points = []
        try:
            with open(self.CSV_FILENAME, 'r') as csvfile:
                reader = csv.reader(csvfile)
                next(reader) 
                for row in reader:
                    raw_points.append([float(row[0]), float(row[1])])
        except FileNotFoundError:
            self.get_logger().error(f"파일 없음: {self.CSV_FILENAME}")
            return

        # === 보간(Interpolation) 로직 ===
        interpolated_points = []
        RESOLUTION = 0.1  # 10cm 간격으로 점을 쪼갬

        for i in range(len(raw_points) - 1):
            p1 = np.array(raw_points[i])
            p2 = np.array(raw_points[i+1])
            
            # 두 점 사이 거리 계산
            dist = np.linalg.norm(p2 - p1)
            
            # 거리에 따라 몇 개의 점을 채울지 계산
            num_points = int(dist / RESOLUTION)
            
            # np.linspace로 점들 생성 (p1에서 p2까지 균등하게)
            # endpoint=False로 해야 다음 구간 시작점과 겹치지 않음
            if num_points > 0:
                new_points = np.linspace(p1, p2, num_points, endpoint=False)
                interpolated_points.extend(new_points)
            else:
                interpolated_points.append(p1)

        # 마지막 점 추가
        interpolated_points.append(raw_points[-1])
        
        self.waypoints = np.array(interpolated_points)
        self.get_logger().info(f"원본 {len(raw_points)}개 -> 보간 후 {len(self.waypoints)}개 로드 완료.")

    def odom_callback(self, msg):
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        curr_yaw = math.atan2(siny_cosp, cosy_cosp)

        # === 도착 확인 ===
        if len(self.waypoints) > 0:
            last_wp = self.waypoints[-1]
            dx = last_wp[0] - curr_x
            dy = last_wp[1] - curr_y
            if math.hypot(dx, dy) < 0.5:
                self.publish_drive(0.0, 0.0)
                self.get_logger().info("도착! 정지합니다.")
                return

        # 1. 목표점 찾기 (로직은 단순한 방식 그대로 유지)
        target_point = self.find_target_point(curr_x, curr_y)

        if target_point is None:
            return

        # 2. 좌표 변환
        tx = target_point[0] - curr_x
        ty = target_point[1] - curr_y
        
        rotated_x = tx * math.cos(-curr_yaw) - ty * math.sin(-curr_yaw)
        rotated_y = tx * math.sin(-curr_yaw) + ty * math.cos(-curr_yaw)

        # 3. 조향각 계산
        if self.LOOKAHEAD_DISTANCE == 0: return
        curvature = 2.0 * rotated_y / (self.LOOKAHEAD_DISTANCE ** 2)
        steering_angle = math.atan(curvature * self.WHEELBASE)

        # 4. 주행 명령
        self.publish_drive(self.VELOCITY, steering_angle)

    def find_target_point(self, x, y):
        # 복잡한 로직 없이 전체 탐색 (보간된 점들이라 매우 촘촘함)
        if len(self.waypoints) == 0:
            return None

        distances = np.linalg.norm(self.waypoints - np.array([x, y]), axis=1)
        nearest_idx = np.argmin(distances)
        
        # 촘촘한 점들을 따라가므로 Target Point가 부드럽게 밀려남
        for i in range(nearest_idx, len(self.waypoints)):
            if distances[i] > self.LOOKAHEAD_DISTANCE:
                return self.waypoints[i]
        
        return self.waypoints[-1]

    def publish_drive(self, speed, angle):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = float(speed)
        drive_msg.drive.steering_angle = float(angle)
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_drive(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()