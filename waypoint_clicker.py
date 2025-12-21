import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import csv
import os

class WaypointClicker(Node):
    def __init__(self):
        super().__init__('waypoint_clicker')
        # RViz의 Publish Point 버튼이 쏘는 토픽을 구독합니다
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.listener_callback,
            10)
        
        self.waypoints = []
        self.get_logger().info('웨이포인트 수집기가 시작되었습니다!')
        self.get_logger().info('RViz에서 "Publish Point"를 누르고 맵에 경로를 순서대로 찍으세요.')
        self.get_logger().info('다 찍은 후 Ctrl+C를 누르면 파일로 저장됩니다.')

    def listener_callback(self, msg):
        x = msg.point.x
        y = msg.point.y
        self.waypoints.append([x, y])
        self.get_logger().info(f'포인트 추가됨: X={x:.2f}, Y={y:.2f} (총 {len(self.waypoints)}개)')

    def save_to_csv(self):
        # 파일 저장 경로 설정 (현재 폴더에 저장)
        filename = 'clicked_waypoints.csv'
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['x', 'y']) # 헤더
            writer.writerows(self.waypoints)
        print(f'\n총 {len(self.waypoints)}개의 웨이포인트가 {filename} 파일에 저장되었습니다.')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointClicker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C를 누르면 저장 함수 실행
        node.save_to_csv()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
