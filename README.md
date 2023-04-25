[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-8d59dc4de5201274e310e4c54b9627a8934c3b88527886e3b421487c677d23eb.svg)](https://classroom.github.com/a/j9y_86cr)

# plansys2_gpsr

## Índice
- [Enunciado](#enunciado)
- [Setup](#setup)
- [Objetivo](#objetivo)
- [Problemas](#problemas)
- [BehaviorTree](#behaviortree)
- [Ejecución](#ejecución)
- [Tests](#tests)
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

## Setup
Compilar el paquete.
```
colcon build --packages-select plansys2_gpsr
```

## Objetivo

## Problemas
PROBLEMA 1
La abuela ha salido a jugar al bingo y ha encargado al robot ordenar la casa. Este deberá recoger varios objetos que hay repartidos por las habitaciones de la casa. En la siguiente imagen se encuentra la disposición del robot y de los objetos.

PROBLEMA 2
La abuela ha tenido un ataque de histeria viendo el salvame y está a punto de lanzarse por la terraza. El robot deberá coger a la abuela a tiempo y encerrarla en su dormitorio. Después, deberá recoger las pastillas repartidas por toda la casa, para después ir al cuarto de la abuela y entregarselas.

PROBLEMA 3
La abuela manda al robot realizar una serie de tareas. Estas serán:
- Coger la regadera del salon, llenarla en la cocina y regar las plantas de la terraza.
- Coger la aspiradora y aspirar el polvo de las habitaciones.
- Coger al perro sucio, lavarlo en el baño, llevarlo a la terraza para que se seque y dejarlo en el salón con la abuela.

## BehaviorTree

## Ejecución
Lanzar el simulador.
```
ros2 launch ir_robots simulation.launch.py
```
Lanzar la navegación.
```
ros2 launch plansys2_gpsr gpsr_launch.py
ros2 run plansys2_gpsr move_action_node
ros2 run plansys2_gpsr gpsr_controller 
```

Para cambiar el goal hay que ir al gpsr_controller y modificar donde pone SetGoal

## Tests

## Videos

## Authors
 - [Jose Manuel](https://github.com/Josetost)
 - [Laura Roa](https://github.com/lroa2019)
 - [Jorge Rando](https://github.com/jorgerando)
 - [Santiago Fenes](https://github.com/santtfg)
