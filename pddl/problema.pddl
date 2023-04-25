(define (problem planning-1)
    (:domain planning)
    (:objects
        Bano Cocina Salon Dormitorio Pasillo - room
        Robot - robot
        PBano PCocina PSalon PDormitorio PPasillo - door
        Medicina Toalla Plato Cubiertos - object
    )
    (:init

        ;Definicion del Mapa

        (ConnectedTo Pasillo Bano PBano)
        (doorAt PBano Bano)
        (ConnectedTo Bano Pasillo PBano)
        (doorAt PBano Pasillo)

        (ConnectedTo Pasillo Cocina PCocina)
        (doorAt PCocina Cocina)
        (ConnectedTo Cocina Pasillo PCocina)
        (doorAt PCocina Pasillo)

        (ConnectedTo Pasillo Salon PSalon)
        (doorAt PSalon Salon)
        (ConnectedTo Salon Pasillo PSalon)
        (doorAt PSalon Pasillo)

        (ConnectedTo Pasillo Dormitorio  PDormitorio)
        (doorAt PDormitorio Dormitorio )
        (ConnectedTo Dormitorio Pasillo PDormitorio)
        (doorAt PDormitorio Pasillo)

        ;Estado Puertas

        (doorOpen PBano)
        (doorOpen PCocina)
        (doorOpen PSalon)
        (doorOpen PDormitorio)
        (doorOpen PPasillo)

        ;Colocacion Robot Objetos (estado inicial objetos)

        (robotAt Robot Pasillo )

        (objectAt Medicina Bano)
        (objectAt Plato Salon)
        (objectAt Cubiertos Salon)
        (objectAt Toalla Dormitorio)

    )

    (:goal
        (and
        (objectAt Medicina Dormitorio)
        (objectAt Plato Cocina)
        (objectAt Cubiertos Cocina)
        (objectAt Toalla Bano)
         )

    )
)