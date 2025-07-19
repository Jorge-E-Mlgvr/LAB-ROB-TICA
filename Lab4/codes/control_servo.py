# pincher_control/control_servo.py
import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
import time

# Direcciones de registro en el AX-12A
ADDR_TORQUE_ENABLE    = 24
ADDR_GOAL_POSITION    = 30
ADDR_MOVING_SPEED     = 32
ADDR_TORQUE_LIMIT     = 34
ADDR_PRESENT_POSITION = 36

class PincherController(Node):

    def __init__(self):
        super().__init__('pincher_controller')

    # Parámetros
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('dxl_ids', [1, 2, 3, 4, 5])
        self.declare_parameter('goal_positions', [512, 512, 512, 512, 512])
        self.declare_parameter('moving_speed', 100)     # 0–1023 
        self.declare_parameter('torque_limit', 800)     # 0–1023 
        self.declare_parameter('delay', 2.0)

        port_name      = self.get_parameter('port').value
        baudrate       = self.get_parameter('baudrate').value
        dxl_ids        = self.get_parameter('dxl_ids').value
        goal_positions = self.get_parameter('goal_positions').value
        moving_speed   = self.get_parameter('moving_speed').value
        torque_limit   = self.get_parameter('torque_limit').value
        delay_seconds  = self.get_parameter('delay').value



        self.dxl_ids = self.get_parameter('dxl_ids').value
        self.goal_positions = self.get_parameter('goal_positions').value
        self.moving_speed = self.get_parameter('moving_speed').value
        self.torque_limit = self.get_parameter('torque_limit').value
        self.delay_seconds = self.get_parameter('delay').value

        self.port_name = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value

        # Inicializar comunicación
        self.port = PortHandler(self.port_name)
        self.port.openPort()
        self.port.setBaudRate(self.baudrate)
        self.packet = PacketHandler(1.0)

        if len(goal_positions) != len(dxl_ids):
            self.get_logger().error(
                f'La lista goal_positions ({len(goal_positions)}) '
                f'debe tener la misma longitud que dxl_ids ({len(dxl_ids)})'
            )
            rclpy.shutdown()
            return

        # Inicializar comunicación
        port   = PortHandler(port_name)
        port.openPort()
        port.setBaudRate(baudrate)
        packet = PacketHandler(1.0)

        # 1) Configurar torque_limit, velocidad y enviar posición a cada servo
        for dxl_id, goal in zip(dxl_ids, goal_positions):
            # Limitar torque
            packet.write2ByteTxRx(port, dxl_id, ADDR_TORQUE_LIMIT, torque_limit)
            # Limitar velocidad
            packet.write2ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, moving_speed)
            # Habilitar torque
            packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 1)
            # Enviar posición objetivo
            packet.write2ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, goal)
            self.get_logger().info(f'[ID {dxl_id}] → goal={goal}, speed={moving_speed}, torque_limit={torque_limit}')
            time.sleep(2)

        # 2) (Opcional) Leer y mostrar posición actual
        for dxl_id in dxl_ids:
            pos, _, _ = packet.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
            self.get_logger().info(f'[ID {dxl_id}] posición actual={pos}')

        # 3) Esperar a que todos los servos alcancen la posición
        self.get_logger().info(f'Esperando {delay_seconds}s para completar movimiento...')
        time.sleep(delay_seconds)

        # 4) Apagar torque en todos los servos
    def terminar(self):
        for dxl_id in self.dxl_ids:
            self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)

        # 5) Cerrar puerto y terminar nodo
        self.port.closePort()
        rclpy.shutdown()


    def cambioPos(self, newPos):
        if len(newPos) != len(self.dxl_ids):
            self.get_logger().error("La nueva posición debe tener la misma longitud que dxl_ids")
            return

        self.goal_positions = newPos

        for dxl_id, goal in zip(self.dxl_ids, self.goal_positions):
            self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_GOAL_POSITION, goal)
            self.get_logger().info(
                f'[ID {dxl_id}] → goal={goal}, speed={self.moving_speed}, torque_limit={self.torque_limit}'
            )
            time.sleep(2)




def deg2pwm(deg):
    pwm = int((deg-180)/(360/1023))
    return pwm

def main(args=None):
    rclpy.init(args=args)
    Pincher = PincherController()
    pos0 = [512, 512, 512, 512, 512] 
    pos1 = [582, 582, 568, 454, 512]
    pos2 = [412, 610, 426, 596, 512]
    pos3 = [753, 454, 667, 582, 512]
    pos4 = [738, 412, 667, 383, 512]

    '''time.sleep(2)
    Pincher.cambioPos(pos1)
    time.sleep(2)
    Pincher.cambioPos(pos2)
    time.sleep(2)
    Pincher.cambioPos(pos3)
    time.sleep(2)
    Pincher.cambioPos(pos4)
    time.sleep(2)
    Pincher.cambioPos(pos0)
    Pincher.terminar()
    #while True:'''
        


    # No es necesario spin() para un movimiento puntual
    # rclpy.spin(node)  # habilita sólo si agregas callbacks


if __name__ == '__main__':
    main()
