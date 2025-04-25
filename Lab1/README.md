# Laboratorio Turtlesim

## Introducción del laboratorio

El objetivo del presente laboratorio es controlar el movimiento de la tortuga generada por `turtlesim` mediante teclado de dos formas distintas:
  1. Con flechas predefinidas de teclado moverla de forma lineal y angular.
  2. Con flechas respectivas a las iniciales nuestras, dibujar una letra predefinida.

Las flechas predefinidas a los movimientos lineales y angulares son "flecha arriba" para avance frontal y "flecha abajo" para retroceso. Por otro lado, "flecha izquierda" será para giro antihorario y "flecha derecha" será para giro horario.
Dado que nuestras iniciales son "J", "E", "M", "G" y "A", las teclas letradas respectivas se usaran para cada una de las figuras personalizadas.

## Procedimiento en términal (Ejecución)

En primer lugar se ubica uno en la carpeta respectiva de ROS 2:

'''base
$ cd ~/ros2_ws
'''

Se prepara el entorno de la terminal para abrir la ventana de la tortuga de la siguiente forma:

```bash
$ source /opt/ros/humble/setup.bash 
```

Donde `source` es un comando built-in de la shell, que ejecuta los comandos dentro de un archivo especificado cuya dirección se especifica en seguida (es decir, el archivo con ruta `/opt/ros/humble/setup.bash`). El archivo `setup.bash`, entonces, es un script que añade las rutas de los ejecutables de ROS 2 (como `ros2`, `colcon`) al path (al terminal para su uso), las rutas de las bibliotecas de ROS 2, las rutas de los paquete de Python para ROS 2.

Se ejecuta otro comando de la shell:

```bash
$ source install/setup.bash
```
El cual es un comando que el script `setup.bash` del directorio `install` del workspace local, y añade las rutas de los ejecutables, bibliotecas, módulos de python del dicho directorio a las variables de entorno modificadas en el anterior comando.

Ya finalmente se puede ejecutar el siguiente comando, el cual crea el nodo de la tortuguita y al muestra en una ventana:

```bash
$ ros2 run turtlesim turtlesim_node
```

En esta línea, `ros2 run` se utiliza para correr un ejecutable (nodo) del workspace, y `turtlesim` es el nombre del paquete de ROS 2 que contiene dicho nodo a ejecutar, en particular el nodo `turtlesim_node`. Al ejecutar este último nodo, se inicia la simulación de la tortuga, apareciendo una ventana gráfica con una tortuga:

![image alt](https://github.com/Jorge-E-Mlgvr/LAB-ROB-TICA/blob/main/Lab1/pictures/turtle1.png?raw=true)

Es necesario ahora tener la capacidad de controlar el movimiento de la tortuga por medio de la ejecución de otro nodo, como por ejemplo `turtle_teleop_key` el cual permite controlar la tortuga con teclas predefinidas. Sin embargo, dado que queremos realizar formas y control particular desde script, se ejecutará un código de Python llamado `move_turtle.py`.

Se ejecuta este último de la siguiente forma, por necesidad en una nueva terminal:

```bash
$ source /opt/ros/humble/setup.bash
$ source install/setup.bash
$ colcon build
$ ros2 run my_turtle_controller move_turtle
```
Obsérvese que se prepara el entorno para ROS 2 de la misma forma que en el otro terminal, y además se ejecuta la herramienta `colcon` con el subcomando `build`, lo cual compila los paquetes de ROS 2 que se encuentran en el espacio de trabajo actual. El paquete `my_turtle_controller` se edita por medio de vsc, y ya habiendo ejecutado esta serie de comandos se puede controlar la tortuga por medio de las teclas indicadas y con el desempeño deseado.

## Estructura del script `move_turtle.py`

```Python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import threading
import time
import math
import sys
import tty
import termios

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.theta=0.0
        self.lock = threading.Lock()
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.actualizar_pose, 10)
        """self.pose = None"""
        self.running = True

        self.input_thread = threading.Thread(target=self.escuchar_teclas)
        self.input_thread.daemon = True
        self.input_thread.start()

    def actualizar_pose(self, msg):
        self.theta = msg.theta
        with self.lock:
            self.theta = msg.theta

    def escuchar_teclas(self):
        print("Controles:")
        print("  j - Dibujar letra J (alineada automáticamente)")
        print("  Flechas - Mover la tortuga (↑ ↓ ← →)")
        print("  q - Salir")
        while self.running:
            tecla = self.get_key()
            if tecla == 'j':
                self.get_logger().info("Dibujando la letra J...")
                self.alinear_tortuga(-math.pi / 2)  # Apuntar hacia abajo
                self.dibujar_letra_j()
            elif tecla == 'e':
                self.get_logger().info("Dibujando la letra E...")
                self.alinear_tortuga(-math.pi / 2)
                self.dibujar_letra_e()
            elif tecla == 'm':
                self.get_logger().info("Dibujando la letra M...")
                self.alinear_tortuga(-math.pi / 2)
                self.dibujar_letra_m()
            elif tecla == 'g':
                self.get_logger().info("Dibujando la letra G...")
                self.alinear_tortuga(-math.pi / 2)
                self.dibujar_letra_g()
            elif tecla == 'a':
                self.get_logger().info("Dibujando la letra A...")
                self.alinear_tortuga(-math.pi / 2)
                self.dibujar_letra_a()        
            elif tecla == '\x1b[A':  # Flecha arriba
                self.mover_tortuga(1.5, 0.0)
            elif tecla == '\x1b[B':  # Flecha abajo
                self.mover_tortuga(-1.5, 0.0)
            elif tecla == '\x1b[C':  # Flecha derecha
                self.mover_tortuga(0.0, -1.5)
            elif tecla == '\x1b[D':  # Flecha izquierda
                self.mover_tortuga(0.0, 1.5)
            elif tecla == 'q':
                print("Saliendo...")
                self.running = False
                rclpy.shutdown()

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch1 = sys.stdin.read(1)
            if ch1 == '\x1b':
                ch2 = sys.stdin.read(1)
                ch3 = sys.stdin.read(1)
                return ch1 + ch2 + ch3
            else:
                return ch1
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def mover_tortuga(self, lin_x, ang_z):
        msg = Twist()
        msg.linear.x = lin_x
        msg.angular.z = ang_z
        self.publisher_.publish(msg)
        time.sleep(0.2)
        self.detener()

    def dibujar_letra_j(self):
        self.mover(1.5, 0.0, 2)      # Hacia abajo
        self.mover(1.0, -1.5, 1.5)   # Curva derecha
        self.mover(0.5, 2.0, 1)      # Gancho
        self.detener()
    
    def dibujar_letra_e(self):
        # Línea vertical
        self.mover_tortuga(10.0,0.0)
        self.detener()
        time.sleep(0.2)

        # Infeior horizontal
        self.alinear_tortuga(0.0)
        self.mover_tortuga(5.0,0.0)
        self.detener()
        time.sleep(0.2)

        # Regresar al medio
        self.mover_tortuga(-4.5, 0.0)
        self.alinear_tortuga(-math.pi / 2)
        self.mover_tortuga(-5.0, 0.0)
        self.alinear_tortuga(0)
        self.mover_tortuga(4.0, 0.0)
        self.detener()
        time.sleep(0.2)

        # Regresar al centro base y trazar línea superior
        self.mover_tortuga(-3.8, 0.0)
        self.alinear_tortuga(-math.pi / 2)
        self.mover_tortuga(-5.0, 0.0)
        self.alinear_tortuga(0)
        self.mover_tortuga(5.0, 0.0)
        self.detener()

    def dibujar_letra_m(self):
        # Línea izquierda
        self.mover_tortuga(10.0, 0.0)
        self.detener()
        time.sleep(0.2)

        # Regresar y dibujar la primera diagonal
        self.mover_tortuga(-10.0, 0.0)
        self.alinear_tortuga(-(math.pi/4))
        self.mover_tortuga(7.0, 0.0)
        self.detener()
        time.sleep(0.2)

        # Segunda diagonal
        self.alinear_tortuga((math.pi/4))
        self.mover_tortuga(7.0, 0.0)
        self.detener()
        time.sleep(0.2)

        # Línea derecha
        self.alinear_tortuga(-(math.pi/2))
        self.mover_tortuga(10.0, 0.0)
        self.detener()

    def dibujar_letra_g(self):
        # C semi-abierto
        self.mover(1.2, -1.0, 2.0)
        self.detener()
        time.sleep(0.2)

        # Hacer gancho de G
        self.mover(0.0, -1.0, 1.0)  # Gira a la derecha
        self.mover(0.8, 0.0, 1.0)   # Línea horizontal interna
        self.detener()

    def dibujar_letra_a(self):
        # Gira y dibuja pierna izquierda
        self.alinear_tortuga(-(math.pi /3))
        self.mover_tortuga(-10.0, 0.0)
        self.detener()
        time.sleep(0.2)

        # Gira y dibuja pierna derecha
        self.alinear_tortuga((math.pi/3))
        self.mover_tortuga(-10.0, 0.0)
        self.detener()
        time.sleep(0.2)

        # Línea horizontal al medio
        self.mover_tortuga(5.0, 0.0)
        self.alinear_tortuga(0)
        self.mover_tortuga(8.0,0.0)
        self.detener()

    def mover(self, lin_x, ang_z, duracion):
        msg = Twist()
        msg.linear.x = lin_x
        msg.angular.z = ang_z
        tiempo_inicial = self.get_clock().now().seconds_nanoseconds()[0]
        while self.get_clock().now().seconds_nanoseconds()[0] - tiempo_inicial < duracion:
            self.publisher_.publish(msg)
            time.sleep(0.1)

    def girar(self, angulo):
        msg = Twist()
        if angulo > 0:
            msg.angular.z = 1.5
        else:
            msg.angular.z = -1.5
        tiempo = abs(angulo) / 1.5
        t0 = time.time()
        while time.time() - t0 < tiempo:
            self.publisher_.publish(msg)
            time.sleep(0.1)
        self.detener()

    def alinear_tortuga(self, angulo_objetivo):
        tolerancia = 0.001
        max_duracion = 15
        velocidad = 0.2

        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < max_duracion:
            with self.lock:
                error = self._normalizar_angulo(angulo_objetivo - self.theta)

            if abs(error) < tolerancia:
                break

            msg = Twist()
            msg.angular.z = velocidad if error > 0 else -velocidad
            self.publisher_.publish(msg)
            time.sleep(0.05)

        self.detener()

    def _normalizar_angulo(self, angulo):
        return math.atan2(math.sin(angulo), math.cos(angulo))
    
    def detener(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
```
