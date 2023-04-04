(define (domain planning)
(:requirements :strips :typing :durative-actions)
(:types
  room
  door
  robot
  object
)

(:predicates 
  (robotat ?r - robot ?l - room)
  (doorclosed ?d - door)
  (dooropen ?d - door)
  (connectedto ?l1 ?l2 - room ?d - door)
  (objectat ?o - object ?l - room)
  (objectatRobot ?r - robot ?o - object)
  (doorat ?d - door ?r - room)
)

(:durative-action move
    :parameters (?robot - robot ?prev_room - room ?next_room - room ?door - door)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robotat ?robot ?prev_room))
        (at start(connectedto ?prev_room ?next_room ?door))
        (at start(dooropen ?door))
    )
    :effect (and 
        (at start(not (robotat ?robot ?prev_room)))
        (at end(robotat ?robot ?next_room))
    )
)

(:durative-action opendoor
    :parameters (?robot - robot ?door - door ?room - room)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robotat ?robot ?room))
        (at start(doorat ?door ?room))
        (at start(doorclosed ?door))
    )
    :effect (and 
        (at start(not (doorclosed ?door)))
        (at end(dooropen ?door))
    )
)

(:durative-action closedoor
    :parameters (?robot - robot ?door - door ?room - room)
    :duration ( = ?duration 5)
    :condition (and 
        (at start(robotat ?robot ?room))
        (at start(doorat ?door ?room))
        (at start(dooropen ?door))
    )
    :effect (and 
        (at start(not (dooropen ?door)))
        (at end(doorclosed ?door))
    )
)

(:durative-action pickitem
    :parameters (?robot - robot ?object - object ?room - room)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robotat ?robot ?room))
        (at start(objectat ?object ?room))
     )
    :effect (and 
        (at start(not(objectat ?object ?room)))
        (at end(objectatRobot ?robot ?object))
    )
)

(:durative-action dropitem
    :parameters (?robot - robot ?object - object ?room - room)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robotat ?robot ?room))
        (at start(objectatRobot ?robot ?object))
     )
    :effect (and 
        (at start(not(objectatRobot ?robot ?object)))
        (at end(objectat ?object ?room))
    )
)

)