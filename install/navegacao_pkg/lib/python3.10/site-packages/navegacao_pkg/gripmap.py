import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import numpy as np

class GridMappingNode(Node):
    def __init__(self):
        super().__init__('grid_mapping_node')
        
        # Tamanho do mapa
        self.map_width = 200  # 20 metros de largura (20 metros * 10 cm por célula)
        self.map_height = 200  # 20 metros de altura (20 metros * 10 cm por célula)
        self.resolution = 0.1  # Resolução do mapa (10 cm por célula)
        
        # Inicia a matriz de mapa com valores desconhecidos (50%)
        self.occupancy_grid = np.full((self.map_height, self.map_width), 50)  # 50% = valor desconhecido
        
        # Assinante do tópico de LaserScan
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',  # Tópico de varredura do LiDAR
            self.laser_callback,
            10)
        
        # Publicador do mapa de ocupação
        self.grid_publisher = self.create_publisher(
            OccupancyGrid,
            '/occupancy_grid',
            10)
    
    def laser_callback(self, msg):
        """Processa os dados de LaserScan para gerar um mapa de ocupação."""
        # Obter as dimensões do mapa (baseado na resolução)
        width = self.map_width
        height = self.map_height
        
        # Inicializar um novo mapa
        grid_map = OccupancyGrid()
        grid_map.header.stamp = self.get_clock().now().to_msg()
        grid_map.header.frame_id = "map"
        grid_map.info.resolution = self.resolution
        grid_map.info.width = width
        grid_map.info.height = height
        grid_map.data = list(self.occupancy_grid.flatten())  # Dados do mapa
        
        # Aqui você pode adicionar lógica para atualizar a ocupação com base nos dados do LaserScan
        
        # Publicar o mapa
        self.grid_publisher.publish(grid_map)

    def update_occupancy(self, laser_data):
        """Atualiza a grade de ocupação a partir dos dados de LaserScan."""
        # Lógica de mapeamento simples, usando ray-casting ou Bresenham para converter
        # as leituras do LiDAR em um mapa de ocupação. Aqui você precisará integrar o
        # seu algoritmo para "marcar" células ocupadas ou livres.
        for i, distance in enumerate(laser_data.ranges):
            # Exemplo simples: marcar as células com base no alcance do LiDAR
            if distance < 5.0:  # Se a distância for menor que 5 metros, consideramos ocupada
                angle = laser_data.angle_min + i * laser_data.angle_increment
                # Calcular a célula (x, y) correspondente no mapa com base no ângulo e distância
                # Este é um exemplo simples, você precisará ajustar para o seu mapa
                x = int(self.map_width / 2 + (distance * np.cos(angle)) / self.resolution)
                y = int(self.map_height / 2 - (distance * np.sin(angle)) / self.resolution)
                
                # Garantir que a célula esteja dentro do limite
                if 0 <= x < self.map_width and 0 <= y < self.map_height:
                    self.occupancy_grid[y, x] = 100  # Marcar como ocupado

def main(args=None):
    rclpy.init(args=args)
    node = GridMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
