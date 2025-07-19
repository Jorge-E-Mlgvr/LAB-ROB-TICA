import rclpy
from rclpy.node import Node
import time

# Esto importa los mensajes y servicios del paquete dynamixel_workbench_msgs
from dynamixel_workbench_msgs.msg import JointCommand
from dynamixel_workbench_msgs.srv import SetPosition

class PincherRosController(Node):

    def __init__(self):
        super().__init__('pincher_ros_controller')
        self.get_logger().info('Nodo PincherRosController iniciado.')

        # Se añaden los nombres en base a las ID's
        self.joint_names = ['waist', 'shoulder', 'elbow', 'wrist', 'gripper']
        self.joint_ids = [1, 2, 3, 4, 5]

        self.publisher_ = self.create_publisher(JointCommand, 'dynamixel_workbench/joint_command', 10)
        self.get_logger().info('Publicador creado para el tópico dynamixel_workbench/joint_command')

        # 2. Crear un cliente para llamar a un servicio (y el servicio SetPosition permite enviar un comando a un solo servo por ID).
        self.set_pos_client = self.create_client(SetPosition, 'dynamixel_workbench/set_position')
        while not self.set_pos_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio /dynamixel_workbench/set_position no disponible, esperando...')
        self.get_logger().info('Cliente de servicio creado para /dynamixel_workbench/set_position')
        
    def send_joint_positions_by_topic(self, positions):

        if len(positions) != len(self.joint_ids):
            self.get_logger().error('El número de posiciones no coincide con el número de articulaciones.')
            return

        msg = JointCommand()
        msg.joint_name = self.joint_names
        msg.position = [float(p) for p in positions]  # Las posiciones deben ser floats
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publicando comando de posición en el tópico.')
        self.get_logger().info(f'Posiciones enviadas: {positions}')

    def set_single_joint_position_by_service(self, joint_id, position):

        request = SetPosition.Request()
        request.id = joint_id
        request.position = position

        future = self.set_pos_client.call_async(request)
        self.get_logger().info(f'Llamando al servicio para mover el ID {joint_id} a la posición {position}')
        
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None and future.result().result:
            self.get_logger().info(f'Servicio ejecutado con éxito para el ID {joint_id}.')
        else:
            self.get_logger().error(f'Fallo al llamar al servicio para el ID {joint_id}.')
            

def main(args=None):
    rclpy.init(args=args)
    controller = PincherRosController()

    # Poses de ejemplo
    home_pose = [512, 512, 512, 512, 512]
    pose_1 = [582, 582, 568, 454, 512]
    
    try:
        # Ejemplo de movimiento usando el tópico (preferido para movimientos secuenciales/continuos)
        controller.get_logger().info('Moviendo a la pose HOME usando el tópico.')
        controller.send_joint_positions_by_topic(home_pose)
        time.sleep(3)
        controller.get_logger().info('Moviendo a la Pose 1 usando el tópico.')
        controller.send_joint_positions_by_topic(pose_1)
        time.sleep(3)

        # Ejemplo de movimiento de un solo servo usando un servicio
        controller.get_logger().info('Moviendo solo el servo del hombro (ID 2) a 600 usando el servicio.')
        controller.set_single_joint_position_by_service(2, 600)
        time.sleep(3)

    except Exception as e:
        controller.get_logger().error(f'Ocurrió un error: {e}')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()