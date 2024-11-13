import cv2
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0

    def update(self, measured_value):
        error = self.setpoint - measured_value
        self.integral += error
        derivative = error - self.previous_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.vel_msg = Twist()  # Inicia a mensagem de movimento
        self.cap = cv2.VideoCapture(1)  # Abertura da câmera
        self.points = [(249, 285), (344, 282), (431, 283)]  # Pontos a serem monitorados
        
        # Inicialização do controlador PID
        self.pid_linear = PIDController(kp=0.1, ki=0.01, kd=0.05, setpoint=150)  # Ajuste os parâmetros conforme necessário
        self.pid_angular = PIDController(kp=0.5, ki=0.01, kd=0.1, setpoint=0)  # Ajuste os parâmetros conforme necessário
        
        self.color_thread = threading.Thread(target=self.color_detection_thread)
        self.color_thread.start()

    def timer_callback(self):
        # Aqui você pode definir a lógica de movimento, que será chamada a cada ciclo do timer
        colors = self.color_detection_thread()
        
        # A lógica de controle PID deve ser aplicada aqui
        linear_error = colors[0][0]  # Exemplo: usar a cor do primeiro ponto como erro linear
        angular_error = colors[1][0]  # Exemplo: usar a cor do segundo ponto como erro angular

        # Atualiza a velocidade linear e angular usando o PID
        self.vel_msg.linear.x = self.pid_linear.update(linear_error)
        self.vel_msg.angular.z = self.pid_angular.update(angular_error)

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

def main():
    rclpy.init()
    robot_controller = RobotController()
    robot_controller.run()
    robot_controller.cap.release()
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main()