import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import math
import csv
import numpy as np
import os

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # =================================================
        # [튜닝 포인트] 이 변수들만 조절하면 됩니다!
        # =================================================
        self.LOOKAHEAD_DISTANCE = 1.5   # (m) 전방 주시 거리
        self.WHEELBASE = 0.33           # (m) 차량 축거 (F1Tenth 표준)
        self.CSV_FILENAME = 'raceline.csv' # 사용할 경로 파일 이름
        
        # 안전을 위한 최대/최소 속도 제한 (CSV 값이 이상할 경우 대비)
        self.MAX_SPEED = 7.0
        self.MIN_SPEED = 0.5
        # =================================================

        # 토픽 설정
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)
        
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)

        # 경로 데이터 저장소 [x, y, velocity]
        self.waypoints = [] 
        
        # CSV 파일 로드
        self.load_waypoints()
        
        self.get_logger().info(f"Pure Pursuit 준비 완료. Lookahead: {self.LOOKAHEAD_DISTANCE}m")

    def load_waypoints(self):
        """ 
        CSV 파일(x, y, speed)을 읽어옵니다.
        이미 촘촘하게 생성된 파일이므로 추가 보간(Interpolation)은 하지 않습니다.
        """
        if not os.path.exists(self.CSV_FILENAME):
            self.get_logger().error(f"파일을 찾을 수 없습니다: {self.CSV_FILENAME}")
            return

        loaded_points = []
        try:
            with open(self.CSV_FILENAME, 'r') as csvfile:
                reader = csv.reader(csvfile)
                # 헤더(첫 줄)가 문자열이면 건너뜀
                header = next(reader, None)
                
                for row in reader:
                    try:
                        # CSV 형식이 x, y, speed 순서라고 가정
                        x = float(row[0])
                        y = float(row[1])
                        v = float(row[2]) if len(row) > 2 else 1.0 # 속도 없으면 1.0 기본값
                        
                        # 속도 제한 적용 (Safety Clip)
                        v = np.clip(v, self.MIN_SPEED, self.MAX_SPEED)
                        
                        loaded_points.append([x, y, v])
                    except ValueError:
                        continue # 숫자가 아닌 행은 무시
                        
            self.waypoints = np.array(loaded_points)
            self.get_logger().info(f"경로 로드 완료: {len(self.waypoints)}개의 포인트")
            
        except Exception as e:
            self.get_logger().error(f"CSV 로드 중 오류 발생: {str(e)}")

    def odom_callback(self, msg):
        # 1. 현재 차량 위치 및 방향(Yaw) 파악
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        
        # 쿼터니언 -> 오일러(Yaw) 변환
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        curr_yaw = math.atan2(siny_cosp, cosy_cosp)

        # 2. 목표점(Target Point) 찾기
        target_data = self.find_target_point(curr_x, curr_y)
        
        if target_data is None:
            return # 목표점이 없으면 주행 안 함

        target_x, target_y, target_v = target_data

        # 3. 차량 기준 좌표계로 변환 (Global -> Local)
        dx = target_x - curr_x
        dy = target_y - curr_y
        
        # 회전 변환 행렬 적용
        local_x = dx * math.cos(-curr_yaw) - dy * math.sin(-curr_yaw)
        local_y = dx * math.sin(-curr_yaw) + dy * math.cos(-curr_yaw)

        # 4. Pure Pursuit 조향각 계산
        # 곡률(Curvature) = 2y / L^2
        curvature = 2.0 * local_y / (self.LOOKAHEAD_DISTANCE ** 2)
        steering_angle = math.atan(curvature * self.WHEELBASE)

        # 5. 주행 명령 발행 (CSV에서 가져온 속도 사용)
        self.publish_drive(target_v, steering_angle)

    def find_target_point(self, x, y):
        """
        현재 위치에서 Lookahead 거리보다 살짝 멀리 있는 점을 찾습니다.
        반환값: [target_x, target_y, target_speed]
        """
        if len(self.waypoints) == 0:
            return None

        # 1. 현재 위치에서 모든 웨이포인트까지의 거리 계산
        # (최적화를 위해 KDTree 등을 쓸 수 있지만, 점이 몇천 개 수준이면 이 방식도 충분히 빠릅니다)
        dx = self.waypoints[:, 0] - x
        dy = self.waypoints[:, 1] - y
        distances = np.hypot(dx, dy)
        
        # 2. 가장 가까운 점의 인덱스 찾기
        nearest_idx = np.argmin(distances)
        
        # 3. Lookahead 거리보다 먼 첫 번째 점 찾기 (Track Forward)
        # nearest_idx부터 시작해서 배열 끝까지 검사
        lookahead_idx = nearest_idx
        
        found = False
        # 배열 끝까지 탐색
        for i in range(nearest_idx, len(self.waypoints)):
            if distances[i] >= self.LOOKAHEAD_DISTANCE:
                lookahead_idx = i
                found = True
                break
        
        # 만약 끝까지 갔는데도 못 찾았으면 (트랙이 루프인 경우) 다시 처음부터 조금 더 탐색
        if not found:
            for i in range(0, nearest_idx):
                if distances[i] >= self.LOOKAHEAD_DISTANCE:
                    lookahead_idx = i
                    break
        
        return self.waypoints[lookahead_idx]

    def publish_drive(self, speed, angle):
        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(angle)
        self.drive_pub.publish(msg)

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