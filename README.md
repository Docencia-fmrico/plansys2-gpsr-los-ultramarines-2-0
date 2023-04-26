# plansys2_gpsr

## Índice
- [Enunciado](#enunciado)
- [Objetivo](#objetivo)
- [Problemas](#problemas)
- [BehaviorTree](#behaviortree)
- [Ejecución](#ejecución)
- [Videos](#videos)
- [Authors](#authors)

## Enunciado
Ejercicio 4 de Planificación y Sistemas Cognitivos 2023

En grupos de 4, haced una aplicación en ROS 2 usando PlanSys2 que use el dominio de la [Práctica 3](https://github.com/Docencia-fmrico/planning-exercise/blob/main/README.md). El robot debe poder realizar en un simulador, navegando con Nav2, goals similares a los siguientes:

(ordena_casa robot1)
(abrir_puerta puerta_principal)
(dar robot1 vaso_leche abuelita)
(dar robot1 medicina abuelita)

Puntuación (sobre 10):   
* +5 correcto funcionamiento en el robot simulado.
* +2 Readme.md bien documentado con videos.
* +2 CI con Test de los nodos BT
* -3 Warnings o que no pase los tests.

## Objetivo
El problema que se plantea en este ejercicio consiste en programar un robot para que realice una serie de tareas en un simulador. Estas tareas incluyen ordenar la casa, abrir la puerta principal y entregar un vaso de leche y medicina a la abuelita.

Para resolver este problema, se debe planificar una secuencia de acciones que permita al robot llevar a cabo estas tareas de manera eficiente y coordinada. Esto implica utilizar herramientas de planificación, como PlanSys2, para generar planes que permitan al robot alcanzar sus objetivos.

Además, para realizar estas tareas, el robot debe ser capaz de navegar por el simulador utilizando Nav2. Esto requiere que el robot tenga un buen conocimiento del entorno y que sea capaz de tomar decisiones inteligentes sobre cómo moverse para alcanzar sus objetivos.

El dominio define 5 durative-action que el robot puede realizar:
- move: mueve el robot de una habitación a otra si la puerta entre las dos habitaciones está abierta.
- opendoor: abre una puerta entre dos habitaciones si el robot está en la habitación donde se encuentra la puerta.
- closedoor: cierra una puerta entre dos habitaciones si el robot está en la habitación donde se encuentra la puerta.
- pickitem: recoge un objeto si el robot está en la habitación donde se encuentra el objeto.
- dropitem: deja un objeto en una habitación si el robot está en esa habitación.

Además, el dominio incluye los siguientes predicados:
- robotat: devuelve true si el robot está en la habitación especificada.
- doorclosed: devuelve true si la puerta especificada está cerrada.
- dooropen: devuelve true si la puerta especificada está abierta.
- connectedto: lo usamos para hacer una conexión entre dos habitaciones y una puerta. Es útil porque nos permite conectar habitaciones.
- objectat: devuelve true si el objeto especificado está en la habitación especificada.
- objectatRobot: devuelve true si el robot lleva el objeto.
- doorat: nos dice dónde está una puerta.

## Problemas
### PROBLEMA 1: CLEAN THE HOUSE
La abuela ha salido a jugar al bingo y ha encargado al robot ordenar la casa. Este deberá recoger varios objetos que hay repartidos por las habitaciones de la casa y depositarlos en el lugar que le corresponden. En la siguiente imagen se encuentra la disposición del robot y de los objetos.

![MAPA1](https://user-images.githubusercontent.com/98589920/234407442-5a882f24-c765-4685-aea3-2650fb156865.png)

### PROBLEMA 2: CRAZY GRANDMA
La abuela ha tenido un ataque de histeria viendo el salvame y está a punto de lanzarse por la terraza. El robot deberá coger a la abuela a tiempo y encerrarla en su dormitorio. Después, deberá recoger las pastillas repartidas por toda la casa y un vaso de agua, para después ir al dormitorio de la abuela y entregarselas. Este problema contará con 2 subgoals: primero salvar a la abuela y encerrarla, y segundo encontrar las pastillas y el agua y entregarlas a la abuela.

![MAPA2](https://user-images.githubusercontent.com/98589920/234391409-b23ef682-58be-4321-9304-0e3b25d2d8ab.png)

### PROBLEMA 3: DOING CHORES
La abuela se va otra vez a jugar al bingo y manda al robot realizar una serie de tareas. Estas serán:
- Coger la regadera del salon, llenarla en la cocina y regar las plantas de la terraza.
- Coger la aspiradora y aspirar el polvo de las habitaciones.
- Coger al perro sucio, lavarlo en el baño, llevarlo a la terraza para que se seque y dejarlo en el salón con la abuela (asumimos que es el mismo perro).

![MAPA3](https://user-images.githubusercontent.com/98589920/234391129-040b6df4-45dd-4246-b3c0-c9ea94f23ce6.png)

## BehaviorTree
- CloseDoor.hpp: Este nodo representa la acción de cerrar una puerta. Este nodo tomará como entrada los parámetros necesarios para identificar la puerta que se debe cerrar. Cuando se ejecuta este nodo, enviará una señal a los actuadores del robot para que cierre la puerta especificada.
- DropItem.hpp: Este nodo representa la acción de soltar un objeto. Este nodo tomará como entrada los parámetros necesarios para identificar el objeto que se debe soltar. Cuando se ejecuta este nodo, enviará una señal a los actuadores del robot para que suelte el objeto especificado.
- OpenDoor.hpp: Este nodo representa la acción de abrir una puerta. Este nodo tomará como entrada los parámetros necesarios para identificar la puerta que se debe abrir. Cuando se ejecuta este nodo, enviará una señal a los actuadores del robot para que abra la puerta especificada.
- PickItem.hpp: Este nodo representa la acción de recoger un objeto. Este nodo tomará como entrada los parámetros necesarios para identificar el objeto que se debe recoger. Cuando se ejecuta este nodo, enviará una señal a los actuadores del robot para que recoja el objeto especificado.
- move.hpp: Este nodo representa la acción de moverse a una ubicación específica. Este nodo tomará como entrada los parámetros necesarios para identificar la ubicación a la que se debe mover el robot. Cuando se ejecuta este nodo, enviará una señal a los actuadores del robot para que se mueva a la ubicación especificada.

## Ejecución
Lanzar el simulador.
```
ros2 launch ir_robots simulation.launch.py
```
Lanzar la navegación.
```
ros2 launch ir_robots navigation.launch.py
```
Lanzar la planificación
```
ros2 launch plansys2_gpsr gpsr_launch.py
ros2 run plansys2_gpsr move_action_node
ros2 run plansys2_gpsr gpsr_controller 
```

Para cambiar el goal hay que ir al gpsr_controller y modificar donde pone SetGoal

## Videos (x5000)

### pr1 ( 22 min )
https://user-images.githubusercontent.com/69701088/234404223-fe1c36c3-1464-4ed8-8699-6e5acc098e49.mp4

### pr2 ( 33 min )

https://user-images.githubusercontent.com/69701088/234433375-6e0f61ad-5098-4c85-bff5-5a8be22fc7eb.mp4

### pr3 ( 1h 10 min )

## Authors
 - [Jose Manuel](https://github.com/Josetost)
 - [Laura Roa](https://github.com/lroa2019)
 - [Jorge Rando](https://github.com/jorgerando)
 - [Santiago Fenes](https://github.com/santtfg)
