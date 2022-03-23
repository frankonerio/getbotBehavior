;========================================================================================================
;========================================================================================================

;   AUTHOR:                        SALEM ALAIDROOS
;   LAST UPDATE:                   19.03.2022 
;   BATTERY MANAGEMENT SYSTEM:
;       BACKGROUND DOMAIN:         ON
;       NAVIGATION DOMAIN:         OFF
;       OBJHANDELING DOMAIN:       ON

;========================================================================================================
;========================================================================================================

(define
    
;================================================================
;================================================================
    
    (domain disaster_drescue_robot)
    (:requirements 
        :typing 
        :negative-preconditions
        :fluents
        :durative-actions
    )

;================================================================
;================================================================

    (:types
        world robot navigation obj                      - object
            destination door stairs elevator            - navigation
            item_victim pipe                            - obj
                ball box victim                         - item_victim
    )
        
;================================================================
;================================================================
    
    (:predicates
        
    ;======================================
    ;Backgorund_Domain
    ;======================================
        
        (map_available ?r - robot ?w - world)
        (exploring ?r - robot ?w - world)
        (mapping ?r - robot ?w - world)
        (obj_detecting ?r - robot ?w - world)
        
        (charger_approached ?r - robot)

    ;======================================
    ;Navigation_Domain
    ;======================================
        
        (no_door_inway ?ds - destination)
        (door_inway ?d - door ?ds - destination)
            (door_opened ?d - door ?ds - destination)
        (stairs_usable ?s - stairs ?w - world)
        (elevator_usable ?e - elevator ?w - world)

        (stairs_approached ?r - robot ?s - stairs)
        (elevator_approached ?r - robot ?e - elevator)

        (destination_reached ?r - robot ?ds - destination)

    ;======================================
    ;ObjHandeling_Domain
    ;======================================
    
        

    )

;================================================================
;================================================================
    
    (:functions
        
    ;======================================
    ;Backgorund_Domain
    ;======================================
        
        (battery_level ?r - robot)

    ;======================================
    ;Navigation_Domain
    ;======================================
        
        (current_floor ?r - robot ?w - world)
        (destination_floor ?ds - destination ?w - world)

    ;======================================
    ;ObjHandeling_Domain
    ;======================================
        


    )

;========================================================================================================
;========================================================================================================
    
    ;======================================
    ;Backgorund_Domain
    ;======================================

    ;==============
    (:durative-action explore_start
        :parameters (?w - world ?r - robot)
        :duration (= ?duration 1)
        :condition (and
            (at start(map_available ?r ?w))
            ;(at start(>= (battery_level ?r) 10))
        )
        :effect (and
            (at end(exploring ?r ?w))
            ;(at end(decrease (batterylevel ?r) 0))
        )
    )
    ;==============
    (:durative-action map_start
        :parameters (?w - world ?r - robot)
        :duration (= ?duration 1)
        :condition (and
            (at start(exploring ?r ?w))
            ;(at start(>= (battery_level ?r) 10))
        )
        :effect (and
            (at end(mapping ?r ?w))
            (at end(map_available ?r ?w))
            ;(at end(decrease (batterylevel ?r) 0))
        )
    )
    ;==============
    (:durative-action wander_around
        :parameters (?w - world ?r - robot)
        :duration (= ?duration 5)
        :condition (and
            ;(at start(>= (battery_level ?r) 15))
        )
        :effect (and
            (at end(map_available ?r ?w))
            ;(at end(decrease (battery_level ?r) 5))
        )
    )
    ;==============
    (:durative-action obj_detect_start
        :parameters (?w - world ?r - robot)
        :duration (= ?duration 1)
        :condition (and
            (at start(exploring ?r ?w))
            ;(at start(>= (battery_level ?r) 10))
        )
        :effect (and
            (at end(obj_detecting ?r ?w))
            ;(at end(decrease (batterylevel ?r) 0))
        )
    )
    ;==============
    (:durative-action charger_approach
        :parameters (?r - robot)
        :duration (= ?duration 5)
        :condition (and
            (at start(>= (battery_level ?r) 10))
        )
        :effect (and
            (at end(charger_approached ?r))
            (at end(decrease (battery_level ?r) 5))
        )
    )
    ;==============
    (:durative-action battery_charge_1
        :parameters (?r - robot)
        :duration (= ?duration 1)
        :condition (and
            (over all(charger_approached ?r))
        )
        :effect (and
            (at start(increase (battery_level ?r) 1))
        )
    )
    ;==============

    ;======================================
    ;Navigation_Domain
    ;======================================

    ;==============
    (:durative-action elevator_approach
        :parameters (?w - world ?r - robot ?e - elevator)
        :duration (= ?duration 5)
        :condition (and
            (at start(elevator_usable ?e ?w)) 
        )
        :effect (and
            (at end(elevator_approached ?r ?e))
    )
    )
    (:durative-action stairs_approach
        :parameters (?w - world ?r - robot ?s - stairs)
        :duration (= ?duration 5)
        :condition (and
            (at start(stairs_usable ?s ?w)) 
        )
        :effect (and
            (at end(stairs_approached ?r ?s))
        )
    )
    ;==============
    (:durative-action elevator_up
        :parameters (?w - world ?r - robot ?e - elevator ?ds - destination)
        :duration (= ?duration 10)
        :condition (and
            (at start(elevator_approached ?r ?e))
            (at start(< (current_floor ?r ?w) (destination_floor ?ds ?w)))
        )
        :effect (and
            (at end(increase (current_floor ?r ?w) 1))
        )
    )
    (:durative-action elevator_down
        :parameters (?w - world ?r - robot ?e - elevator ?ds - destination)
        :duration (= ?duration 10)
        :condition (and
            (at start(elevator_approached ?r ?e))
            (at start(> (current_floor ?r ?w) (destination_floor ?ds ?w)))
        )
        :effect (and
            (at end(decrease (current_floor ?r ?w) 1))
        )
    )
    ;==============
    (:durative-action stairs_up
        :parameters (?w - world ?r - robot ?s - stairs ?ds - destination)
        :duration (= ?duration 10)
        :condition (and
            (at start(stairs_approached ?r ?s))
            (at start(< (current_floor ?r ?w) (destination_floor ?ds ?w)))
        )
        :effect (and
            (at end(increase (current_floor ?r ?w) 1))
        )
    )
    (:durative-action stairs_down
        :parameters (?w - world ?r - robot ?s - stairs ?ds - destination)
        :duration (= ?duration 10)
        :condition (and
            (at start(stairs_approached ?r ?s))
            (at start(> (current_floor ?r ?w) (destination_floor ?ds ?w)))
        )
        :effect (and
            (at end(decrease (current_floor ?r ?w) 1))
        )
    )
    ;==============
    (:durative-action door_open
        :parameters (?w - world ?r - robot ?d - door ?ds - destination)
        :duration (= ?duration 2)
        :condition (and
            (at start(door_inway ?d ?ds))
            (at start(>= (current_floor ?r ?w) (destination_floor ?ds ?w)))
            (at start(<= (current_floor ?r ?w) (destination_floor ?ds ?w)))
        )
        :effect (and
            (at end(door_opened ?d ?ds))
        )
    )
    (:durative-action door_destination_navigate
        :parameters (?w - world ?r - robot ?ds - destination ?d - door)
        :duration (= ?duration 5)
        :condition (and
            (at start(door_opened ?d ?ds))
            (at start(>= (current_floor ?r ?w) (destination_floor ?ds ?w)))
            (at start(<= (current_floor ?r ?w) (destination_floor ?ds ?w)))
        )
        :effect (and
            (at end(destination_reached ?r ?ds))
        )
    )
    ;==============
    (:durative-action destination_navigate
        :parameters (?w - world ?r - robot ?ds - destination)
        :duration (= ?duration 5)
        :condition (and
            (at start(no_door_inway ?ds))
            (at start(>= (current_floor ?r ?w) (destination_floor ?ds ?w)))
            (at start(<= (current_floor ?r ?w) (destination_floor ?ds ?w)))
        )
        :effect (and
            (at end(destination_reached ?r ?ds))
        )
    )
    ;==============

    ;======================================
    ;ObjHandeling_Domain
    ;======================================

    
    
;========================================================================================================
;========================================================================================================
)