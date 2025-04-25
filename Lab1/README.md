# Laboratorio Turtlesim

## Introducción del laboratorio

El objetivo del presente laboratorio es controlar el movimiento de la tortuga generada por _turtlesim_ mediante teclado de dos formas distintas:
  1. Con flechas predefinidas de teclado moverla de forma lineal y angular.
  2. Con flechas respectivas a las iniciales nuestras, dibujar una letra predefinida.

Las flechas predefinidas a los movimientos lineales y angulares son "flecha arriba" para avance frontal y "flecha abajo" para retroceso. Por otro lado, "flecha izquierda" será para giro antihorario y "flecha derecha" será para giro horario.
Dado que nuestras iniciales son "J", "E", "M", "G" y "A", las teclas letradas respectivas se usaran para cada una de las figuras personalizadas.

## Procedimiento completo

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
