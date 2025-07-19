# pincher_control/control_servo.py
import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
import time

# Direcciones de registro del motor Dynamixel AX-12A
ADDR_TORQUE_ENABLE    = 24
ADDR_GOAL_POSITION    = 30
ADDR_MOVING_SPEED     = 32
ADDR_TORQUE_LIMIT     = 34
ADDR_PRESENT_POSITION = 36

class PincherController(Node):

    def __init__(self):
        super().__init__('pincher_controller')

        # Parámetros del robot
        self.dxl_ids = [1, 2, 3, 4, 5]
        self.moving_speed = 100
        self.torque_limit = 800
        self.delay_seconds = 2.0

        # Parámetros de conexión
        self.port_name = '/dev/ttyUSB0'
        self.baudrate = 1000000

        # Inicializar comunicación con los servomotores
        self.port = PortHandler(self.port_name)
        self.packet = PacketHandler(1.0)
        
        try:
            self.port.openPort()
            self.port.setBaudRate(self.baudrate)
            self.get_logger().info('Conexión con el robot establecida.')

            # Habilitar torque en todos los servos
            for dxl_id in self.dxl_ids:
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 1)
                self.get_logger().info(f'Torque habilitado en ID {dxl_id}.')

        except Exception as e:
            self.get_logger().error(f'Error al conectar con el robot: {e}')
            self.terminar()

    def cambioPos(self, new_positions):
        """
        Envía posiciones de forma secuencial a cada servomotor.
        """
        if len(new_positions) != len(self.dxl_ids):
            self.get_logger().error("La nueva posición debe tener la misma longitud que los IDs de los motores.")
            return

        for dxl_id, goal in zip(self.dxl_ids, new_positions):
            self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_GOAL_POSITION, goal)
            self.get_logger().info(f'[ID {dxl_id}] → goal={goal}')
            time.sleep(1)  # Pequeña espera entre cada movimiento

    def terminar(self):
        """
        Deshabilita el torque y cierra el puerto de comunicación.
        """
        self.get_logger().info('Apagando servos y cerrando conexión.')
        for dxl_id in self.dxl_ids:
            self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
        self.port.closePort()
        rclpy.shutdown()

    def deg2pwm(self, deg):
        """
        Convierte grados a valores PWM (0-1023).
        """
        pwm = int((deg-180)/(360/1023))
        return pwm