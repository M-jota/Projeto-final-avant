#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class DetectorMangueiraELinha(Node):
    def __init__(self):
        super().__init__('node_visao_desafio')
        self.bridge = CvBridge()

        # SUBSCRIBER da camera 
        self.image_sub = self.create_subscription(Image, '/camera/image', self.image_callback, 10)

        # PUBLISHER para posicao da mangueira (erro normalizado + altura fixa soh para referencia)
        self.posicao_mang_pub = self.create_publisher(Point, '/posicao_mangueira', 10)
        
        # PUBLISHER para indicar se estah detectando ou nao a mangueira
        self.deteccao_mang_pub = self.create_publisher(Bool, '/deteccao_mangueira', 10)

        # PUBLISHER para mostrar a centralizacao
        self.centralizado_mang_pub = self.create_publisher(Bool, '/centralizada_mangueira', 10)

        # PUBLISHER que envia o erro de alinhamento em relação a linha azul
        self.erro_linha_pub = self.create_publisher(Point, '/erro_linha_azul', 10)
        
        # PUBLISHER para indicar se estah detectando ou nao a linha azul no frame atual
        self.linha_detectada_pub = self.create_publisher(Bool, '/deteccao_linha_azul', 10)

        self.fixed_z = 2.0  # altura fixa do drone soh para referencia

        self.get_logger().info("No da visao comp iniciado.")

    def image_callback(self, msg):
        # converte imagem ROS p/ opencv
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # converte o frame para hsv
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)



        #----mangueira-------

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



        #------linha azul------
        # mascara para detectar azul
        lower_blue = np.array([90, 80, 50])
        upper_blue = np.array([130, 255, 255])
        mascara_azul = cv2.inRange(hsv, lower_blue, upper_blue)

        # pega regiao inferior da imagem (ROI - regiao de interesse)
        # roi eh usado para focar apenas nas partes que importam
        # nesse caso, como a linha estah no chao, foi selecionada parte inferior
        roi = mascara_azul[int(altura*0.6):altura, :]

        M_azul = cv2.moments(roi) # pega os momentos dentro da selecao da roi
        erro_linha = Point()
        linha_detectada = Bool()

        if M_azul["m00"] > 5000:   # limite maior porque linha eh larga, segundo enunciado: 25cm
            cx_azul = int(M_azul["m10"] / M_azul["m00"])

            # erro normalizado
            erro_norm = (cx_azul - largura / 2) / (largura / 2)

            erro_linha.x = float(erro_norm)
            erro_linha.y = 0.0
            erro_linha.z = 0.0
            linha_detectada.data = True
        
        else:
            # nao identifica a linha
            erro_linha.x = float('nan')
            erro_linha.y = float('nan')
            erro_linha.z = float('nan')
            linha_detectada.data = False

        # publica
        self.erro_linha_pub.publish(erro_linha)
        self.linha_detectada_pub.publish(linha_detectada)


        # mostra imagem
        cv2.imshow("camera drone", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DetectorMangueiraELinha()
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
