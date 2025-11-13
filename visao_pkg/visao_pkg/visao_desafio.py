#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class DetectorMangueira(Node):
    def __init__(self):
        super().__init__('node_visao_desafio')
        self.bridge = CvBridge()

        # SUBSCRIBER da camera 
        self.image_sub = self.create_subscription(Image, '/camera/image', self.image_callback, 10)

        # PUBLISHER para posicao da mangueira (erro normalizado + altura fixa soh para referencia)
        self.posicao_mang_pub = self.create_publisher(Point, '/posicao_mangueira', 10)
        
        # PUBLISHER para mostrar se estah detectando ou nao a mangueira
        self.deteccao_mang_pub = self.create_publisher(Bool, '/deteccao_mangueira', 10)

        # PUBLISHER para mostrar a centralizacao
        self.centralizado_mang_pub = self.create_publisher(Bool, '/centralizada_mangueira', 10)

        self.fixed_z = 2.0  # altura fixa do drone soh para referencia

        self.get_logger().info("No da visao comp iniciado.")

    def image_callback(self, msg):
        # converte imagem ROS p/ opencv
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # converte o frame para hsv
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # mascara para vermelho (duas faixas)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mascara1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mascara2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mascara = cv2.bitwise_or(mascara1, mascara2)
        #mascara = mascara1 + mascara2

        # calcula o centro da mascara
        # formula do centro: cx = [m10]/[m00] e cy = [m01][m00]
        M = cv2.moments(mascara)
        altura, largura, _ = frame.shape

        ponto = Point()
        detectado = Bool()
        centralizado = Bool()

        if M['m00'] > 200: # 200 para evitar deteccao de pequenos ruidos dentro da faixa do vermelho
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # erro normalizado (-1 a 1)
            error_x = (cx - largura / 2) / (largura / 2)
            error_y = (cy - altura / 2) / (altura / 2)

            ponto.x = float(error_x)
            ponto.y = float(error_y)
            ponto.z = self.fixed_z
            detectado.data = True

            # verifica centralizacao: o valor absoluto do erro tem que estar o mais proximo de 0
            centralizado.data = abs(error_x) < 0.05 and abs(error_y) < 0.05

        else:
            # nao identifica a mangueira
            ponto.x = float('nan')
            ponto.y = float('nan')
            ponto.z = float('nan')
            detectado.data = False
            centralizado.data = False

        # publica as mensagens que vao ser usadas pela navegacao
        self.posicao_mang_pub.publish(ponto)
        self.deteccao_mang_pub.publish(detectado)
        self.centralizado_mang_pub.publish(centralizado)

        # mostra imagem
        cv2.imshow("deteccao da mangueira vermelha", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DetectorMangueira()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
