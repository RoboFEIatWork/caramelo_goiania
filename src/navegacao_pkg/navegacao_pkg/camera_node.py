import rclpy
from rclpy.node import Node
import cv2
import threading
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.cap = cv2.VideoCapture(0)  # Abre a câmera, use 0 ou 1 dependendo do dispositivo
        self.timer = self.create_timer(0.1, self.show_camera_feed)  # Configura a taxa de atualização
        self.points = [(250, 285), (350, 285), (450, 285)]  # Pontos a serem monitorados
    

        # Usando BGR (Blue,Green,Red)
    def timer_callback(self,original):

        # # ANDA PARA FRENTE BAB
        # if (colors[0] >= np.array([140, 140, 140])).all() and colors[1][0] >= 110 and colors[1][1] <= 40 and (colors[2] >= np.array([140, 140, 140])).all():
        #     # self.vel_msg.linear.x = 0.2  # Anda para frente
        #     self.get_logger().info("Anda para frente")

        # # GIRA PARA ESQUERDA AAB
        # elif colors[0][0] >= 110 and colors[0][1] <= 40 and colors[1][0] >= 110  and colors[1][1] <= 40 and (colors[2] >= np.array([140, 140, 140])).all():
        #     # self.vel_msg.angular.z = 0.2  # Girar para a esquerda
        #     self.get_logger().info("Girar para a esquerda")

        # # GIRA PARA DIREITA BAA
        # elif (colors[0] >= np.array([140, 140, 140])).all() and colors[1][0] >= 110 and colors[1][1] <= 40 and colors[2][0] >= 110 and colors[2][1] <= 40:
        #     # self.vel_msg.angular.z = -0.2  # Girar para a direita   
        #     self.get_logger().info("Girar para a direita")

        # else:
        #     # self.vel_msg.linear.x = 0.0  # Parar de ir para frente
        #     # self.vel_msg.angular.z = 0.2  # Gira para esquerda buscar uma linha
        #     self.get_logger().info("Gira para esquerda buscar uma linha")

        hsv = cv2.cvtColor(original,cv2.COLOR_BGR2HSV)

        #Determina o range do blue
        lower_blue = np.array([90,100,50]) #limite inferior
        upper_blue = np.array([110,255,255]) #limite superior

        mask = cv2.inRange(hsv,lower_blue,upper_blue)

        res = cv2.bitwise_and(original,original,mask = mask)

        # Exibe o quadro capturado
        # cv2.imshow("Camera Feed", res)

        return res

    def identify_colors(self, frame):
        colors = []                                     
        for point in self.points:
            color = frame[point[1], point[0]]
            colors.append(color)
        return colors
    
    def detect_line_direction(self, frame):
        frame[:] = [0,0,0]
        for point in self.points:
            cv2.circle(frame, point, radius=5, color=(255, 255, 255), thickness=-1)
        return frame

    def show_camera_feed(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Falha ao capturar a imagem da câmera.")
            return
        
        # Identifica as cores nos pontos
        colors = self.identify_colors(frame)

        # Desenha os pontos na imagem
        frame_with_points = self.detect_line_direction(frame)

        self.get_logger().info(f"Colors {colors}")


        #Define o movimento dependendo das cores encontradas

        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

        #Determina o range do blue
        lower_blue = np.array([90,100,50]) #limite inferior
        upper_blue = np.array([110,255,255]) #limite superior

        mask = cv2.inRange(hsv,lower_blue,upper_blue)

        res = cv2.bitwise_and(frame,frame,mask = mask)

        # Exibe o quadro capturado
        cv2.imshow("Camera Feed", res)
        
        # Sai ao pressionar 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()  # Encerra o nó se 'q' for pressionado

    def destroy_node(self):
        # Libera a câmera e fecha todas as janelas
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
