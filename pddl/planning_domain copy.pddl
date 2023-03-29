(define (domain planning)
(:requirements :strips :typing :negative-preconditions :disjunctive-preconditions)
(:types
  room
  door
  robot
  object
)

(:predicates
  ;; Position of the robot
  (robotAt ?r - robot ?l - room)

  ;; Door closed
  (doorClosed ?d - door)
  (DoorClosedHighPriority ?d - door)

  ;; Door open
  (doorOpen ?d - door)
  (OpenDoorHighPriority ?d - door)

  ;; Room is conected by a door
  (ConnectedTo ?l1 ?l2 - room ?d - door)

  ;; Position of an object
  (objectAt ?o - object ?l - room)
  (objectAtHighPriority ?o - object ?l - room)
  (objectAtRobot ?r - robot ?o - object)

  ;; Position of the door
  (doorAt ?d - door ?r - room)

  ;; If HighPriority actions are done
  (ObjectAtHighPriorityCompleted)
  (DoorClosedHighPriorityCompleted)
  (OpenDoorHighPriorityCompleted)
)

;; High priority position of an object completed
(:action object_at_high_priority
    :parameters (?o - object ?r - room)
    :precondition (and
        (objectAtHighPriority ?o ?r)
     )
    :effect (and
        (ObjectAtHighPriorityCompleted)
     )
)

;; High priority door closed completed
(:action DoorClosedHighPriority_p
    :parameters (?d - door)
    :precondition (and
        (DoorClosedHighPriority ?d)
     )
    :effect (and
        (DoorClosedHighPriorityCompleted)
     )
)

;; High priority door open completed
(:action OpenDoorHighPriority_p
    :parameters (?d - door)
    :precondition (and
        (OpenDoorHighPriority ?d)
     )
    :effect (and
        (OpenDoorHighPriorityCompleted)
     )
)

;; ------------------------------------------------------- MOVE ACTIONS -------------------------------------------------------

(:action move_location_high_priority
    :parameters (?r - robot ?prev_room - room ?next_room - room ?d - door)
    :precondition
    (and
        (not
            (and
                (not(not(DoorClosedHighPriorityCompleted)))
                (not(not(OpenDoorHighPriorityCompleted)))
                (not(not(ObjectAtHighPriorityCompleted)))
            )
        )
        
        (robotAt ?r ?prev_room)
        (ConnectedTo ?prev_room ?next_room ?d)
        (not
            (and
                (not(OpenDoorHighPriority ?d))
                (not(doorOpen ?d))
            )
        )
    )
    :effect (and 
        (not (robotAt ?r ?prev_room))
        (robotAt ?r ?next_room)
    )
)

(:action move_location
    :parameters (?r - robot ?prev_room - room ?next_room - room ?d - door)
    :precondition 
    (and
        (ObjectAtHighPriorityCompleted)
        (DoorClosedHighPriorityCompleted)
        (OpenDoorHighPriorityCompleted)
        (robotAt ?r ?prev_room)
        (ConnectedTo ?prev_room ?next_room ?d)
        (not
            (and
                (not(OpenDoorHighPriority ?d))
                (not(doorOpen ?d))
            )
        )
    )
    :effect (and 
        (not (robotAt ?r ?prev_room))
        (robotAt ?r ?next_room)
    )
)

;; ------------------------------------------------------- OPEN DOOR ACTIONS -------------------------------------------------------

(:action OpenDoorHighPriority
    :parameters (?r - robot ?d - door ?h - room)
    :precondition (and
        (not
            (and
                (not(not(OpenDoorHighPriorityCompleted)))
                (not(not(DoorClosedHighPriorityCompleted)))
                (not(not(ObjectAtHighPriorityCompleted)))
            )
        )
        (robotAt ?r ?h)
        (doorAt ?d ?h)
        (not
            (and
                (not(doorClosed ?d))
                (not(DoorClosedHighPriority ?d))
            )
        )
        
    )
    :effect (and 
        (not (doorClosed ?d))
        (not (DoorClosedHighPriority ?d))
        (OpenDoorHighPriority ?d)
        (doorOpen ?d)
    )
)

(:action open_door
    :parameters (?r - robot ?d - door ?h - room)
    :precondition (and
        (ObjectAtHighPriorityCompleted)
        (DoorClosedHighPriorityCompleted)
        (OpenDoorHighPriorityCompleted)
        (robotAt ?r ?h)
        (doorAt ?d ?h)
        (not
            (and
                (not(doorClosed ?d))
            )
        )
    )
    :effect (and 
        (not (doorClosed ?d))
        (not (DoorClosedHighPriority ?d))
        (OpenDoorHighPriority ?d)
        (doorOpen ?d)
    )
)

;; ------------------------------------------------------- CLOSE DOOR ACTIONS -------------------------------------------------------

(:action close_door_high_priority
    :parameters (?r - robot ?d - door ?h - room)
    :precondition (and 
        (not
            (and
                (not(not(OpenDoorHighPriorityCompleted)))
                (not(not(DoorClosedHighPriorityCompleted)))
                (not(not(ObjectAtHighPriorityCompleted)))
            )
        )
        (robotAt ?r ?h)
        (doorAt ?d ?h)
        (not
            (and
                (not(OpenDoorHighPriority ?d))
            )
        )
        
    )
    :effect (and 
        (not (doorOpen ?d))
        (not(OpenDoorHighPriority ?d))
        (DoorClosedHighPriority ?d)
        (doorClosed ?d)
    )
)

(:action close_door
    :parameters (?r - robot ?d - door ?h - room)
    :precondition (and 
        (ObjectAtHighPriorityCompleted)
        (DoorClosedHighPriorityCompleted)
        (OpenDoorHighPriorityCompleted)
        (robotAt ?r ?h)
        (doorAt ?d ?h)
        (not
            (and
                (not(doorOpen ?d))
            )
        )
    )
    :effect (and 
        (not (doorOpen ?d))
        (not(OpenDoorHighPriority ?d))
        (DoorClosedHighPriority ?d)
        (doorClosed ?d)
    )
)

;; ------------------------------------------------------- PICK OBJECTS ACTIONS -------------------------------------------------------

(:action pick_object_high_priority
    :parameters (?r - robot ?o - object ?h - room)
    :precondition (and
        (not
            (and
                (not(not(OpenDoorHighPriorityCompleted)))
                (not(not(DoorClosedHighPriorityCompleted)))
                (not(not(ObjectAtHighPriorityCompleted)))
            )
        )
        (robotAt ?r ?h)
        (not
            (and
                (not(objectAtHighPriority ?o ?h))
            )
        )
        
     )
    :effect (and 
        (not(objectAtHighPriority ?o ?h))
        (not(objectAt ?o ?h))
        (objectAtRobot ?r ?o)
    )
)

(:action pick_object
    :parameters (?r - robot ?o - object ?h - room)
    :precondition (and
        (ObjectAtHighPriorityCompleted)
        (DoorClosedHighPriorityCompleted)
        (OpenDoorHighPriorityCompleted)
        (robotAt ?r ?h)
        (not
            (and
                (not(objectAtHighPriority ?o ?h))
                (not(objectAt ?o ?h))
            )
        )
     )
    :effect (and 
        (not(objectAtHighPriority ?o ?h))
        (not(objectAt ?o ?h))
        (objectAtRobot ?r ?o)
    )
)

;; ------------------------------------------------------- DROP OBJECT ACTIONS -------------------------------------------------------

(:action drop_object_high_priority
    :parameters (?r - robot ?o - object ?h - room)
    :precondition (and
        (not
            (and
                (not(not(OpenDoorHighPriorityCompleted)))
                (not(not(DoorClosedHighPriorityCompleted)))
                (not(not(ObjectAtHighPriorityCompleted)))
            )
        )
        (robotAt ?r ?h)
        (objectAtRobot ?r ?o)
     )
    :effect (and 
        (not(objectAtRobot ?r ?o))
        (objectAtHighPriority ?o ?h)
        (objectAt ?o ?h)
    )
)

(:action drop_object
    :parameters (?r - robot ?o - object ?h - room)
    :precondition (and
        (ObjectAtHighPriorityCompleted)
        (DoorClosedHighPriorityCompleted)
        (OpenDoorHighPriorityCompleted)
        (robotAt ?r ?h)
        (objectAtRobot ?r ?o)
     )
    :effect (and 
        (not(objectAtRobot ?r ?o))
        (objectAt ?o ?h)
    )
)

)