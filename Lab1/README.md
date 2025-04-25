# Laboratorio Turtlesim

## Introducción del laboratorio

El objetivo del presente laboratorio es controlar el movimiento de la tortuga generada por _turtlesim_ mediante teclado de dos formas distintas:
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

Donde 'source' es un comando built-in de la shell, que ejecuta los comandos dentro de un archivo especificado cuya dirección se especifica en seguida (es decir, el archivo con ruta "/opt/ros/humble/setup.bash"). El archivo 'setup-bash', entonces, es un script que añade las rutas de los ejecutables de ROS 2 (como 'ros2', 'colcon') al path (al terminal para su uso), las rutas de las bibliotecas de ROS 2, las rutas de los paquete de Python para ROS 2.

Se ejecuta otro comando de la shell:

```bash
$ source install/setup.bash
```
El cual es un comando que el script 'setup.bash' del directorio 'install' del workspace local, y añade las rutas de los ejecutables, bibliotecas, módulos de python del dicho directorio a las variables de entorno modificadas en el anterior comando.

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


