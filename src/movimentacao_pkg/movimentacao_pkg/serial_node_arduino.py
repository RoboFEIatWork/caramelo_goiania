import os
import json
import serial
import serial.tools.list_ports
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.logging import LoggingSeverity

from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class Ros2Serial(Node):
    def __init__(self, node_name="ros2serial_node"):
        super().__init__(node_name=node_name)
        
        self.get_logger().set_level(LoggingSeverity.INFO)
        
        # Parâmetros da porta serial
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.baud_rate = self.get_parameter('baud_rate').value
        self.serial_port = self.get_parameter('serial_port').value
        self.ser = self.connect_to_serial(self.serial_port)
        
        # Envia o comando de reset para a ESP32 assim que o nó ROS 2 inicia
        if self.ser:
            self.ser.write(b'R')  # Envia 'R' como exemplo de comando de reset; adapte se necessário
        
        # Parâmetros do robô
        self.wheel_radius = 0.05  # Raio da roda em metros
        self.lx = 0.2355  # Distância entre rodas no eixo X
        self.ly = 0.15  # Distância entre rodas no eixo Y
        self.pulses_per_rotation = 1024
        self.motor_reduction = 28

        # Estado inicial do robô
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.perf_counter()

        # Distâncias percorridas pelas rodas
        self.encoder_left_front = 0
        self.encoder_right_front = 0
        self.encoder_left_rear = 0
        self.encoder_right_rear = 0

        # Configurações de tópicos ROS 2
        qos_profile = QoSProfile(depth=50)
        self.encoder_pub = self.create_publisher(Int32MultiArray, '/robot_base/encoders', qos_profile)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile)
        
        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            qos_profile
        )
        
        # Thread para publicar mensagens e ler da serial
        self.publish_thread = threading.Thread(target=self.publish_messages)
        self.publish_thread.daemon = True  # Garante que a thread será encerrada com o nó
        self.publish_thread.start()

        


    def connect_to_serial(self, port):
        while rclpy.ok():
            try:
                self.ser = serial.Serial(port, self.baud_rate)
                self.get_logger().info(f"Conexão bem-sucedida com a porta serial {port}")
                self.ser.write(b'\x05')
                return self.ser
            except serial.SerialException as e:
                self.get_logger().error(f"Falha ao conectar com a porta serial: {e}")
            time.sleep(0.5)

    def listener_callback(self, msg):
        data = {
            'linear_x': float(msg.linear.x),
            'linear_y': float(msg.linear.y),
            'linear_z': float(msg.linear.z),
            'angular_x': float(msg.angular.x),
            'angular_y': float(msg.angular.y),
            'angular_z': float(msg.angular.z)
        }

        json_data = json.dumps(data)
        try:
            if self.ser.isOpen():
                self.ser.write(json_data.encode('ascii'))
                self.ser.flush()
                self.get_logger().info("Comando de velocidade enviado com sucesso")
        except (serial.SerialException, serial.SerialTimeoutException) as e:
            self.get_logger().warn(f"Erro ao enviar comando: {e}")

    def publish_messages(self):
        while rclpy.ok():
            try:
                if self.ser and self.ser.is_open:
                    line = self.ser.readline().decode("utf-8").strip()
                    if line:
                        data = json.loads(line)
                        if "encoders" in data:
                            self.get_logger().info("Publicando odometria")
                            self.publish_odometry(data)
                        else:
                            self.get_logger().info("Dados do encoder não recebidos")
            except (serial.SerialException, serial.SerialTimeoutException) as e:
                self.get_logger().warn(f"Conexão serial perdida: {e}")
                self.ser = self.connect_to_serial(self.serial_port)
            except Exception as e:
                self.get_logger().error(f"Erro inesperado: {e}")

    def publish_odometry(self, data):
        encoders = self.get_encoder_values(data)
        current_time = time.perf_counter()
        delta_time = current_time - self.last_time

        # self.get_logger().info(f"Tipo de self.x: {type(self.x)}")
        # self.get_logger().info(f"Tipo de self.y: {type(self.y)}")
        self.get_logger().info(f"delta_time: {delta_time}")
        self.compute_odometry(encoders)
        
        # Atualizando self.x e self.y como float
        self.x = float(self.x)
        self.y = float(self.y)

        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2)
        
        self.odom_pub.publish(odom_msg)
        self.get_logger().info(f"Odometry publicada: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}")

        self.last_time = current_time


    def get_encoder_values(self, data):
        try:

            self.encoder_left_front  = float(data["encoders"][0])
            self.encoder_right_front = - float(data["encoders"][1])
            self.encoder_left_rear   = float(data["encoders"][2])
            self.encoder_right_rear  = - float(data["encoders"][3])
            self.get_logger().info(f"self.encoder_left_front {self.encoder_left_front}")
        except (ValueError, KeyError) as e:
            self.get_logger().error(f"Erro ao processar encoders: {e}")
            return 0, 0, 0, 0  # Valores default caso o erro aconteça
        return (self.encoder_left_front, self.encoder_right_front, self.encoder_left_rear, self.encoder_right_rear)

    def compute_odometry(self, encoders):

        wheel_distances = [(rotacao * 2 * math.pi * self.wheel_radius) for rotacao in encoders]
        self.get_logger().info(f"Distancia percorrida pela roda {wheel_distances}")

        # Cálculo do deslocamento (média das distâncias percorridas pelas rodas)
        avg_distance = sum(wheel_distances) / 4.0

        # Cálculo da mudança na posição
        delta_x = avg_distance * math.cos(self.theta)  # Movimento no eixo x
        delta_y = avg_distance * math.sin(self.theta)  # Movimento no eixo y

        self.get_logger().info(f"Delta x {delta_x}")
        self.get_logger().info(f"Delta y {delta_y}")

        if delta_x < 0.03 : delta_x = 0
        if delta_y < 0.03 : delta_y = 0
        # Atualiza a posição do robo


        # Não calculamos a velocidade angular (wz) aqui, pois você só quer a pose
        # O cálculo de theta (orientação) depende da diferença de distâncias entre as rodas
        # Aqui, podemos usar uma média ponderada para uma aproximação da orientação
        self.theta = (
            (-wheel_distances[0] + wheel_distances[1] - wheel_distances[2] + wheel_distances[3])
            / (4.0 * (self.lx + self.ly))
        )

        self.get_logger().info(f"Delta thetha {self.theta}")


def main(args=None):
    rclpy.init(args=args)
    node = Ros2Serial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupção por teclado')
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
