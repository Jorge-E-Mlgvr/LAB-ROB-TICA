# Laboratorio RobotStudio

<p align="center">
  <img src="picture/pastral.png" alt="Pastel ejemplo" height="400">
</p>

## Introducción del laboratorio
---

El objetivo del presente laboratorio es por medio de simulación y RAPID en un ABB IRB-140:
  1. Diseñar un portaherramienta para el manipulador y emplearlo en el laboratorio.
  2. Dibujar el primer nombre de cada integrante, de forma separada, sobre una torta con un tamaño correspondiente a 20 personas aproximadamente.
  3. Hacer el objetivo anterior de forma que primero se lleve la torta sobre una banda transportadora, la desplace hasta el robot, y tan pronto este útlimo termine el dibujo desplace la torta otra vez con la banda transportadora.

En relación a estos objetivos, se hacen unas observaciones adicionales. Primero, debido a limitaciones de tiempo y disponibilidad del laboratorio, se restringe el movimiento de la banda transportadora solo a la simulación de RobotStudio. Por otra parte, este laboratorio inició con tres integrantes, pero poco tiempo después de su inició el tercer compañero abandonó; se realizó parte del código RAPID para dibujar el nombre del compañero pero no se continuó, debido a la innecesidad. Finalmente, se adecuó el espacio de dibujo en la prueba real y por los implementos usados se dio cierta inclinación que influye en el dibujo final, que se explicará su motivo en uno de los apartados siguientes. Por último, ni se emplea una torta real y tampoco un extrusor de confitura, sino un caja/tablero que hace de torta (con las dimensiones proporcionales) y un marcador que hace de extrusor.

## Procedimiento

### Preliminares
---

Se dispone de las siguientes herramientas en el laboratorio:
  - Un manipulador industrial IRB-140 de la marca ABB.
<p align="center">
  <img src="picture/irb-140.png" alt="Manipulador">
</p>

  - Un controlador IRC5 con un módulo de distribución de energía 3HAC025917-001/00 DSQC 652.
  - Un RobotTeach Pendent de ABB modelo 3HACO28357—001 para el controlador IRC5.

<p align="center">
  <img src="picture/irc5.jpg" alt="Controlador y HMI">
</p>

De los anteriores implementos, solo se deben tener en cuenta dos en particular: el controlador y el manipulador. Esto debido a que con sus modelos de computador se podrá trabajar en el módulo de RAPID dentro del workspace de RobotStudio. Dentro de RobotStudio se incluyen estos implementos, pero todavía hace falta el portaherramienta. El portaherramienta se puede obtener en físico dado un costo y su modelo también, aunque puede que encontrarlo tenga su dificultad; sin embargo, dados los requerimientos del laboratorio y facilidad de producción (manufactura aditiva), se diseña uno propio.


### Diseño del portaherramienta
---

Para el diseño de la herramienta, se consiguen marcadores comunes de la marca Pelikan y se extraen sus medidas haciendo uso de un pie de rey:

<p align="center">
  <img src="picture/medidas_marcador.jpg" alt="Marcador" height="500">
</p>

Donde la medida coloreada en azul es de interés. Lo anterior se debe a que se busca emplear un resorte en la base del marcador como método de tolerancia para el dibujo con el marcador sobre el tablero, de forma que esa medida de la punta al cuerpo del portaherramienta (que va hasta el cuello del cuerpo del marcador) va a ir variando. Las demás medidas, en rojo, servirán para las dimensiones de encaje del marcador en el portaherramienta y su ajuste seguro.

El siguiente es un dibujo CAD, solo ilustriativo, del portaherramienta:

<p align="center">
  <img src="picture/portaherramienta_medidas.jpg" alt="Portaherramienta" height="300">
</p>

<p align="center">
  <img src="picture/portaherramienta_modelo.jpg" alt="Portaherramienta_2" height="300">
</p>

Antes de proceder, se destaca lo siguiente para el procedimiento: es posible dentro de RobotStudio definir paths a partir de puntos flotantes en el mismo entorno, pero es muy inexacto o bien consumiría , para dibujar los nombres. Existe, sin embargo, una ruta alternativa, más sencilla, rápida y exacta: importar un modelo de caja con los nombres impresos en ellas, y utilizar las herramientas de cursor y snap a esquinas/curvas/aristas para realizar paths exactos o por lo menos aproximados. Se realiza la siguiente imprenta sobre un modelo .sat de caja 20x20x10 (mm):

<p align="center">
  <img src="picture/nombres.jpg" alt="Nombres_imprenta" height="300">
</p>

Ya teniendo la herramienta en Inventor, se exporta el modelo y se importa ahora como .sat a RobotStudio:


Antes e

### Presentación

[Link al video de la presentación en youtube.](https://youtu.be/41Mnf04L6z8)
