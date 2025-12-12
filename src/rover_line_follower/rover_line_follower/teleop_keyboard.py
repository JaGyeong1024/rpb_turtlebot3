#!/usr/bin/env python3
"""
키보드로 로버를 제어하는 Teleop 노드
WASD 키 사용
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        # cmd_vel 퍼블리셔
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 속도 설정
        self.linear_speed = 1.0  # m/s
        self.angular_speed = 1.0  # rad/s
        
        # 터미널 설정 저장
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Teleop Keyboard Node Started')
        self.get_logger().info('WASD 키로 로버를 제어하세요:')
        self.get_logger().info('  W: 전진')
        self.get_logger().info('  S: 후진')
        self.get_logger().info('  A: 좌회전')
        self.get_logger().info('  D: 우회전')
        self.get_logger().info('  Space: 정지')
        self.get_logger().info('  Q: 종료')
        
    def get_key(self):
        """키 입력 받기 (논블로킹)"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run(self):
        """메인 루프"""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                twist = Twist()
                
                if key == 'w' or key == 'W':
                    # 전진
                    twist.linear.x = self.linear_speed
                    twist.angular.z = 0.0
                    self.get_logger().info('전진')
                elif key == 's' or key == 'S':
                    # 후진
                    twist.linear.x = -self.linear_speed
                    twist.angular.z = 0.0
                    self.get_logger().info('후진')
                elif key == 'a' or key == 'A':
                    # 좌회전
                    twist.linear.x = 0.0
                    twist.angular.z = self.angular_speed
                    self.get_logger().info('좌회전')
                elif key == 'd' or key == 'D':
                    # 우회전
                    twist.linear.x = 0.0
                    twist.angular.z = -self.angular_speed
                    self.get_logger().info('우회전')
                elif key == ' ':
                    # 정지
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info('정지')
                elif key == 'q' or key == 'Q':
                    # 종료
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
                    self.get_logger().info('종료')
                    break
                else:
                    # 알 수 없는 키는 무시
                    continue
                
                self.publisher_.publish(twist)
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            # 터미널 설정 복원
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            # 정지 명령 전송
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

