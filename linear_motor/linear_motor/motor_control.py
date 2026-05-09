#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from gpiozero import Motor

class LinearMotorNode(Node):
    def __init__(self):
        super().__init__('linear_motor_node')
        
        # GPIO 핀 설정 (IN1=17, IN2=27)
        self.motor = Motor(forward=17, backward=27)
        
        # '/linear' 토픽 구독 설정
        self.subscription = self.create_subscription(
            Int32,
            '/linear',
            self.listener_callback,
            10) # 큐 사이즈
        
        self.get_logger().info('/linear 토픽을 대기')

    def listener_callback(self, msg):
        command = msg.data
        
        if command == 1:
            self.get_logger().info('명령: 상승 (Forward)')
            self.motor.forward()
        elif command == -1:
            self.get_logger().info('명령: 하강 (Backward)')
            self.motor.backward()
        elif command == 0:
            self.get_logger().info('명령: 정지 (Stop)')
            self.motor.stop()
        else:
            self.get_logger().warn(f'알 수 없는 명령: {command}')

    def destroy_node(self):
        # 종료 시 모터 정지 안전장치
        self.motor.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LinearMotorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 노드가 종료됩니다.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()