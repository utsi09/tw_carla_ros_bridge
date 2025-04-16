#!/usr/bin/env python3
"""
sensor_toggle_node.py

ROS2 노드로 동작하면서, 키 입력(예: 숫자 '4')을 받아서 CARLA의 센서 (여기서는 LIDAR)를 토글(켜기/끄기)합니다.
이 코드는 CARLA의 Python API와 pygame을 사용합니다.
"""

import rclpy
from rclpy.node import Node
import carla
import pygame
from pygame.locals import K_4, K_ESCAPE, K_q
import random

def get_vehicle_by_role(world, role_name):
    """
    CARLA world에서 'vehicle.*' 필터로 검색한 후, 
    각 차량 액터의 'role_name' attribute가 주어진 role_name과 일치하는 첫 번째 차량을 반환합니다.
    """
    vehicles = world.get_actors().filter('vehicle.*')
    for v in vehicles:
        if v.attributes.get('role_name', '').lower() == role_name.lower():
            return v
    return None

def get_actor(world, filter_str):
    """Helper: world에서 필터로 조회하여 첫 번째 액터 반환"""
    actors = world.get_actors().filter(filter_str)
    if len(actors) > 0:
        return actors[0]
    return None

class SensorToggleNode(Node):
    def __init__(self):
        super().__init__('sensor_toggle_node')
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        
        host = self.get_parameter('host').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        
        # CARLA 클라이언트 연결
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        
        # 에고 차량 찾기: role_name이 'hero'인 차량을 찾음
        self.vehicle = get_vehicle_by_role(self.world, 'hero')
        if self.vehicle is None:
            self.get_logger().error("No vehicle found with role name 'hero' in the world. Exiting.")
            rclpy.shutdown()
            return
        
        self.blueprint_library = self.world.get_blueprint_library()
        self.lidar_actor = None
        
        # Pygame 초기화 (노드 내에서 키보드 이벤트를 처리)
        pygame.init()
        self.screen = pygame.display.set_mode((800, 600))
        pygame.display.set_caption("CARLA Sensor Toggle (ROS2 Node)")
        self.clock = pygame.time.Clock()
        
        # 타이머: 주기적으로 이벤트 루프 실행
        self.timer = self.create_timer(0.05, self.event_loop)

    def event_loop(self):
        self.clock.tick(30)  # 30 FPS 고정
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.shutdown()
            elif event.type == pygame.KEYDOWN:
                if event.key in (K_ESCAPE, K_q):
                    self.shutdown()
                elif event.key == K_4:
                    self.toggle_lidar()
        self.screen.fill((0, 0, 0))
        pygame.display.flip()

    def toggle_lidar(self):
        if self.lidar_actor and self.lidar_actor.is_listening:
            self.get_logger().info("Stopping LIDAR sensor publication.")
            # 센서의 콜백을 중지해서 데이터를 발행하지 않도록 합니다.
            self.lidar_actor.stop()
        elif self.lidar_actor and not self.lidar_actor.is_listening:
            self.get_logger().info("Starting LIDAR sensor publication.")
            # 센서를 다시 시작합니다. 콜백 함수 lidar_callback은 사용자가 정의한 함수입니다.
            self.lidar_actor.listen(self.lidar_callback)
        else:
            self.get_logger().info("Spawning LIDAR sensor.")
            try:
                lidar_bp = self.blueprint_library.find('sensor.lidar.ray_cast')
                # 속성 설정 (필요에 따라 조정)
                lidar_bp.set_attribute('channels', '32')
                lidar_bp.set_attribute('points_per_second', '320000')
                lidar_bp.set_attribute('range', '50')
                # 차량에 부착할 transform (예: 차량 중앙 위 2.4m)
                lidar_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=2.4))
                self.lidar_actor = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)
                # 센서 데이터를 처리할 콜백 함수 등록 (사용자 정의)
                self.lidar_actor.listen(self.lidar_callback)
            except Exception as e:
                self.get_logger().error(f"Error spawning LIDAR: {e}")
                self.lidar_actor = None

    def shutdown(self):
        self.get_logger().info("Shutting down SensorToggleNode...")
        if self.lidar_actor:
            try:
                self.lidar_actor.destroy()
            except Exception as e:
                self.get_logger().error(f"Error during shutdown: {e}")
        pygame.quit()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SensorToggleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
