import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool


# --- Coordenadas ---
TARGET_POSE = {
    "x": -2.93,
    "y": -9.71,
    "z": 2.44  
}

# posição de decolagem
TAKEOFF_POSE = {
    "x": 0.0,
    "y": 0.0,
    "z": 2.5  
}

TOLERANCIA_DISTANCIA = 0.5 

# --- ESTADOS ---
ESTADO_INICIAL = 0
ESTADO_AGUARDANDO_CONEXAO = 1
ESTADO_REQUISITANDO_MODO = 2
ESTADO_ARMANDO = 3
ESTADO_DECOLAGEM = 4
ESTADO_NAVEGACAO = 5
ESTADO_FINALIZADO = 6
ESTADO_CENTRALIZANDO = 7


class NavegacaoSimplesNode(Node):

    def __init__(self):
        super().__init__('navegacao_simples_node')

        self.estado_atual = ESTADO_INICIAL
        self.pose_atual = None
        self.mavros_conectado = False

        # --- Dados da visão ---
        self.erro_x = 0.0
        self.deteccao = False
        self.centralizado = False

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers MAVROS
        self.odometry_subscriber = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.odometry_callback,
            qos_profile
        )

        self.state_subscriber = self.create_subscription(
            State, 
            '/mavros/state',
            self.state_callback,
            qos_profile
        )

        # --- SUBSCRIBERS DA VISÃO ---
        self.create_subscription(Point, '/posicao_mangueira', self.vision_position_callback, 10)
        self.create_subscription(Bool, '/deteccao_mangueira', self.vision_detection_callback, 10)
        self.create_subscription(Bool, '/centralizada_mangueira', self.vision_center_callback, 10)

        # Publishers MAVROS
        self.setpoint_publisher = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        # Publica que chegou ao alvo para iniciar a centralização
        self.gancho_centralizacao_pub = self.create_publisher(
            Bool,
            '/missao/gancho_centralizar',
            10
        )

        # Serviços
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Aguarda serviços
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço "/mavros/cmd/arming" não disponível, esperando..')

        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço "/mavros/set_mode" não disponível, esperando...')

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.setpoint_timer = self.create_timer(0.05, self.publicar_setpoint_continuo)

        self.setpoint_desejado = PoseStamped()
        self.estado_atual = ESTADO_AGUARDANDO_CONEXAO

    # --- Callbacks da visão ---

    def vision_position_callback(self, msg):
        self.erro_x = msg.x

    def vision_detection_callback(self, msg):
        self.deteccao = msg.data

    def vision_center_callback(self, msg):
        self.centralizado = msg.data

    # --- MAVROS Callbacks ---

    def state_callback(self, msg):
        if not self.mavros_conectado and msg.connected:
            self.get_logger().info('MAVROS conectado!')
            self.mavros_conectado = True
        self.modo_atual = msg.mode
        self.armado = msg.armed

    def odometry_callback(self, msg):
        self.pose_atual = {
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z
        }

    # --- Utilidades ---
    def calcular_distancia(self, pose1, pose2):
        if pose1 is None or pose2 is None:
            return float('inf')
        return math.sqrt(
            (pose1['x'] - pose2['x'])**2 +
            (pose1['y'] - pose2['y'])**2 +
            (pose1['z'] - pose2['z'])**2
        )

    def set_pose_desejada(self, x, y, z):
        self.setpoint_desejado.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_desejado.pose.position.x = float(x)
        self.setpoint_desejado.pose.position.y = float(y)
        self.setpoint_desejado.pose.position.z = float(z)

    # --- MAVROS ---
    def publicar_setpoint_continuo(self):
        self.setpoint_publisher.publish(self.setpoint_desejado)

    def chamar_servico_set_mode(self):
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        self.set_mode_client.call_async(req)

    def chamar_servico_arming(self):
        req = CommandBool.Request()
        req.value = True
        self.arming_client.call_async(req)

    # --- MÁQUINA DE ESTADOS ---

    def timer_callback(self):

        if self.pose_atual is None:
            return

        if not self.mavros_conectado:
            self.set_pose_desejada(self.pose_atual['x'], self.pose_atual['y'], self.pose_atual['z'])
            return

        # ESTADO 1
        if self.estado_atual == ESTADO_AGUARDANDO_CONEXAO:
            self.chamar_servico_set_mode()
            self.estado_atual = ESTADO_REQUISITANDO_MODO
            self.set_pose_desejada(self.pose_atual['x'], self.pose_atual['y'], self.pose_atual['z'])

        # ESTADO 2
        elif self.estado_atual == ESTADO_REQUISITANDO_MODO:
            if self.modo_atual == 'OFFBOARD':
                self.chamar_servico_arming()
                self.estado_atual = ESTADO_ARMANDO
            self.set_pose_desejada(self.pose_atual['x'], self.pose_atual['y'], self.pose_atual['z'])

        # ESTADO 3
        elif self.estado_atual == ESTADO_ARMANDO:
            if self.armado:
                self.estado_atual = ESTADO_DECOLAGEM
            self.set_pose_desejada(self.pose_atual['x'], self.pose_atual['y'], self.pose_atual['z'])

        # ESTADO 4
        elif self.estado_atual == ESTADO_DECOLAGEM:
            self.set_pose_desejada(TAKEOFF_POSE['x'], TAKEOFF_POSE['y'], TAKEOFF_POSE['z'])

            if self.calcular_distancia(self.pose_atual, TAKEOFF_POSE) < TOLERANCIA_DISTANCIA:
                self.estado_atual = ESTADO_NAVEGACAO

        # ESTADO 5
        elif self.estado_atual == ESTADO_NAVEGACAO:
            self.set_pose_desejada(TARGET_POSE['x'], TARGET_POSE['y'], TARGET_POSE['z'])

            if self.calcular_distancia(self.pose_atual, TARGET_POSE) < TOLERANCIA_DISTANCIA:
                self.estado_atual = ESTADO_CENTRALIZANDO
                msg = Bool()
                msg.data = True
                self.gancho_centralizacao_pub.publish(msg)

        # ESTADO 7 – CENTRALIZANDO via visão
        elif self.estado_atual == ESTADO_CENTRALIZANDO:

            if not self.deteccao:
                self.get_logger().warn("Mangueira ainda não detectada.")
                return

            # Ajuste lateral baseado no erro_x
            ganho = 0.8
            correcao_x = -ganho * self.erro_x

            self.set_pose_desejada(
                TARGET_POSE['x'] + correcao_x,
                TARGET_POSE['y'],
                TARGET_POSE['z']
            )

            if self.centralizado:
                self.get_logger().info("CENTRALIZAÇÃO CONCLUÍDA!")
                self.estado_atual = ESTADO_FINALIZADO

        # ESTADO 6
        elif self.estado_atual == ESTADO_FINALIZADO:
            self.set_pose_desejada(TARGET_POSE['x'], TARGET_POSE['y'], TARGET_POSE['z'])


def main(args=None):
    rclpy.init(args=args)
    node = NavegacaoSimplesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()