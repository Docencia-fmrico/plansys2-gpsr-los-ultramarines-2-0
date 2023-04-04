(define (domain planning)
(:requirements :strips :typing)
(:types
  room
  door
  robot
  object
)

(:predicates 
  (robotAt ?r - robot ?l - room)
  (doorClosed ?d - door)
  (doorOpen ?d - door)
  (ConnectedTo ?l1 ?l2 - room ?d - door)
  (objectAt ?o - object ?l - room)
  (objectAtRobot ?r - robot ?o - object)
  (doorAt ?d - door ?r - room)
)

(:action move_location
    :parameters (?robot - robot ?prev_room - room ?next_room - room ?door - door)
    :precondition (and
        (robotAt ?robot ?prev_room)
        (ConnectedTo ?prev_room ?next_room ?door)
        (doorOpen ?door)
    )
    :effect (and 
        (not (robotAt ?robot ?prev_room))
        (robotAt ?robot ?next_room)
    )
)

(:action open_door
    :parameters (?robot - robot ?door - door ?room - room)
    :precondition (and
        (robotAt ?robot ?room)
        (doorAt ?door ?room)
        (doorClosed ?door)
    )
    :effect (and 
        (not (doorClosed ?door))
        (doorOpen ?door)
    )
)

(:action close_door
    :parameters (?robot - robot ?door - door ?room - room)
    :precondition (and 
        (robotAt ?robot ?room)
        (doorAt ?door ?room)
        (doorOpen ?door)
    )
    :effect (and 
        (not (doorOpen ?door))
        (doorClosed ?door)
    )
)

(:action pick_object
    :parameters (?robot - robot ?object - object ?room - room)
    :precondition (and
        (robotAt ?robot ?room)
        (objectAt ?object ?room)
     )
    :effect (and 
        (not(objectAt ?object ?room))
        (objectAtRobot ?robot ?object)
    )
)

(:action drop_object
    :parameters (?robot - robot ?object - object ?room - room)
    :precondition (and
        (robotAt ?robot ?room)
        (objectAtRobot ?robot ?object)
     )
    :effect (and 
        (not(objectAtRobot ?robot ?object))
        (objectAt ?object ?room)
    )
)

)