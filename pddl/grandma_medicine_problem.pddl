(define (problem grandma_medicine_problem)
    (:domain planning)
    (:objects
        Bano Cocina Salon Dormitorio Pasillo - room
        Robot - robot
        PBano PCocina PSalon PDormitorio PPasillo - door
        PastillaVerde PastillaRoja PastillaAmarilla VasitoConAgua - object
    )
    (:init
        (not(ObjectAtHighPriorityCompleted))
        (DoorClosedHighPriorityCompleted)
        (OpenDoorHighPriorityCompleted)
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
        (doorClosed PCocina)
        (doorClosed PSalon)
        (doorOpen PDormitorio)
        (doorClosed PPasillo)

        ;Colocacion Robot Objetos (estado inicial objetos)

        (robotAt Robot Dormitorio )

        (objectAtHighPriority PastillaRoja Bano)
        (objectAt PastillaVerde Salon)
        (objectAt PastillaAmarilla Pasillo)
        (objectAt VasitoConAgua Cocina)

    )

    (:goal
        (and

        (doorClosed PDormitorio)
        (objectAtHighPriority PastillaRoja Dormitorio)
        (objectAt PastillaVerde Dormitorio)
        (objectAt PastillaAmarilla Dormitorio)

        )

    )
)