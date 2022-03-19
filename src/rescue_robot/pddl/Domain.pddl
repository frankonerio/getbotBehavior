;========================================================================================================
;========================================================================================================
;   AUTHOR:         SALEM ALAIDROOS
;   LAST UPDATE:    17.03.2022 


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
            world robot navigation obj     - object
                item_victim pipe           - obj
                    ball box victim        - item_victim
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
            (at start(>= (battery_level ?r) 10))
        )
        :effect (and
            (at end(exploring ?r ?w))
            ;(at end(decrease (batterylevel ?r) 5))
        )
    )
    ;==============
    (:durative-action map_start
        :parameters (?w - world ?r - robot)
        :duration (= ?duration 1)
        :condition (and
            (at start(exploring ?r ?w))
            (at start(>= (battery_level ?r) 10))
        )
        :effect (and
            (at end(mapping ?r ?w))
            (at end(map_available ?r ?w))
            ;(at end(decrease (batterylevel ?r) 5))
        )
    )
    ;==============
    (:durative-action wander_around
        :parameters (?w - world ?r - robot)
        :duration (= ?duration 5)
        :condition (and
            (at start(>= (battery_level ?r) 15))
        )
        :effect (and
            (at end(map_available ?r ?w))
            (at end(decrease (battery_level ?r) 5))
        )
    )
    ;==============
    (:durative-action obj_detect_start
        :parameters (?w - world ?r - robot)
        :duration (= ?duration 1)
        :condition (and
            (at start(exploring ?r ?w))
            (at start(>= (battery_level ?r) 10))
        )
        :effect (and
            (at end(obj_detecting ?r ?w))
            ;(at end(decrease (batterylevel ?r) 5))
        )
    )

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

    ;======================================
    ;Navigation_Domain
    ;======================================

    ;======================================
    ;ObjHandeling_Domain
    ;======================================

;========================================================================================================
;========================================================================================================
)