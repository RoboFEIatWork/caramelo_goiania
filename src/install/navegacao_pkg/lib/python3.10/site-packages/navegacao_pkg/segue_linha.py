import cv2
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class RobotController(Node):



    def __init__(self):
        super().__init__('segue_linha')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.vel_msg = Twist()  # Inicia a mensagem de movimento
        self.cap = cv2.VideoCapture(0)  # Abertura da câmera
        self.points = [(249, 285), (344, 282), (431, 283)]  # Pontos a serem monitorados
        self.color_thread = threading.Thread(target=self.color_detection_thread)
        self.color_thread.start()

     
    def timer_callback(self):
        # Aqui você pode definir a lógica de movimento, que será chamada a cada ciclo do timer
        colors = self.color_detection_thread()
        
        # ANDA PARA FRENTE
        if colors[0] >= [150, 150, 150] and colors[1][2] >= [0, 0, 110] and colors[0] >= [150, 150, 150]:
            self.vel_msg.linear.x = 0.2  # Anda para frente

        # GIRA PARA ESQUERDA
        elif colors[1][2] >= [0, 0, 110] and colors[1][2] >= [0, 0, 110] and colors[0] >= [150, 150, 150]:
            self.vel_msg.angular.z = 0.2  # Girar para a esquerda

        # GIRA PARA DIREITA
        elif colors[0] >= [150, 150, 150] and colors[1][2] >= [0, 0, 110] and colors[1][2] >= [0, 0, 110]:
            self.vel_msg.angular.z = -0.2  # Girar para a direita   

        else:
            self.vel_msg.linear.x = 0.0  # Parar de ir para frente
            self.vel_msg.angular.z = 0.2  # Gira para esquerda buscar uma linha

        self.cmd_vel_pub.publish(self.vel_msg)

    def identify_colors(self, frame):
        colors = []
        for point in self.points:
            color = frame[point[1], point[0]]
            colors.append(color)
        print("Cores nos pontos:", ", ".join(f"{color}" for color in colors))
        return colors

    def detect_line_direction(self, frame, colors):
        for point in self.points:
            cv2.circle(frame, point, radius=5, color=(255, 255, 255), thickness=-1)
        return frame

    def color_detection_thread(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Falha ao capturar a imagem da câmera.")
                break

            # Identifica as cores nos pontos
            colors = self.identify_colors(frame)

            # Desenha os pontos na imagem
            frame_with_points = self.detect_line_direction(frame, colors)

            # Exibe a imagem
            cv2.imshow("Camera with Points", frame_with_points)

            # Sai ao pressionar 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
            return colors

    def run(self):
        # Roda a aplicação ROS2
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()  # Corrigido para usar RobotController
    try:
        robot_controller.run()  # Usar o método run do RobotController
    except KeyboardInterrupt:
        print('Interrupção por teclado capturada.')
    finally:
        robot_controller.cap.release()  # Libera a câmera
        robot_controller.destroy_node()
        