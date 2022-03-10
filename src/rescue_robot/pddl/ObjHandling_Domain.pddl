;====================================================
;====================================================
;AUTHOR: SALEM ALAIDROOS
;====================================================
;====================================================
(define
    ;====================================================
    ;==================================================== 
    (domain DDR_objHandling)
    (:requirements 
        :typing 
        :negative-preconditions
        :fluents
        :durative-actions
    )
    ;====================================================
    ;====================================================

    (:types
        world robot obj - object
            item_victim pipe           - obj
                ball box victim        - item_victim
    )
    ;====================================================
    ;====================================================

    (:predicates
        (obj_approached ?r - robot ?o - obj)

        (item_moved ?r - robot ?iv - item_victim)
        (victim_rescued ?r - robot ?v - victim)
            (item_victim_grabbed ?r - robot ?iv - item_victim)
            (item_victim_target_approached ?r - robot ?iv - item_victim)
            (item_victim_dropped ?r - robot ?iv - item_victim)
        
        (charger_approached ?r - robot)
)   

    (:functions
        (item_victim_detected ?iv - item_victim)
            (item_victim_carried ?r - robot)
            (item_victim_at_target ?iv - item_victim)
            (item_victim_at_target_goal ?iv - item_victim)

        (battery_level ?r - robot)
    )
    
;====================================================
;====================================================
    (:durative-action obj_approach
        :parameters (?w - world ?r - robot ?iv - item_victim)
        :duration (= ?duration 5)
        :condition (and
            (at start(item_victim_dropped ?r ?iv))
            (at start(> (battery_level ?r) 15))
        )
        :effect (and
            (at end(obj_approached ?r ?iv))
            (at end(not(item_victim_target_approached ?r ?iv)))
            (at end(decrease (battery_level ?r) 5))
            (at end(not(charger_approached ?r)))
        )
    )
;====================================================
    (:durative-action item_victim_grab
        :parameters (?w - world ?r - robot ?iv - item_victim)
        :duration (= ?duration 5)
        :condition (and
            (at start(obj_approached ?r ?iv))
            (at start(< (item_victim_carried ?r) 1))
            (at start(> (battery_level ?r) 13))
        )
        :effect (and
            (at end(item_victim_grabbed ?r ?iv))
            (at end(not(item_victim_dropped ?r ?iv)))
            (at end(decrease (item_victim_detected ?iv) 1))
            (at start(increase (item_victim_carried ?r) 1))
            (at end(decrease (battery_level ?r) 3))
        )
    )
;====================================================
    (:durative-action target_approach
        :parameters (?w - world ?r - robot ?iv - item_victim)
        :duration (= ?duration 5)
        :condition (and
            (at start(item_victim_grabbed ?r ?iv))
            (at start(> (item_victim_carried ?r) 0))
            (at start(> (battery_level ?r) 15))
        )
        :effect (and
            (at end(not(obj_approached ?r ?iv)))
            (at end(item_victim_target_approached ?r ?iv))
            (at end(decrease (battery_level ?r) 5))
            (at end(not(charger_approached ?r)))
        )
    )
;====================================================
    (:durative-action item_victim_drop
        :parameters (?w - world ?r - robot ?iv - item_victim)
        :duration (= ?duration 5)
        :condition (and
            (at start(item_victim_grabbed ?r ?iv))
            (at start(> (item_victim_carried ?r) 0))
            (at start(item_victim_target_approached ?r ?iv))
            (at start(> (battery_level ?r) 12))
        )
        :effect (and
            (at end(item_victim_dropped ?r ?iv))
            (at end(not(item_victim_grabbed ?r ?iv)))
            (at start(increase (item_victim_at_target ?iv) 1))
            (at start(decrease (item_victim_carried ?r) 1))
            (at end(decrease (battery_level ?r) 2))
        )
    )
;====================================================
    (:durative-action item_move
        :parameters (?w - world ?r - robot ?iv - item_victim)
        :duration (= ?duration 0.001)
        :condition (and
            (at start(item_victim_dropped ?r ?iv))
            (at start(< (item_victim_carried ?r) 1))
            (at start(> (item_victim_at_target ?iv) (item_victim_at_target_goal ?iv)))
        )
        :effect (and
            (at end(item_moved ?r ?iv))
        )
    )
    (:durative-action victim_rescue
        :parameters (?w - world ?r - robot ?v - victim)
        :duration (= ?duration 0.001)
        :condition (and
            (at start(item_victim_dropped ?r ?v))
            (at start(< (item_victim_carried ?r) 1))
            (at start(> (item_victim_at_target ?v) (item_victim_at_target_goal ?v)))
        )
        :effect (and
            (at end(victim_rescued ?r ?v))
        )
    )
;====================================================
;====================================================
    (:durative-action charger_approach
        :parameters (?r - robot  ?iv - item_victim)
        :duration (= ?duration 5)
        :condition (and
            (at start(> (battery_level ?r) 9))
        )
        :effect (and
            (at end(charger_approached ?r))
            (at end(decrease (battery_level ?r) 10))

            (at end(not(obj_approached ?r ?iv)))
            (at end(not(item_victim_target_approached ?r ?iv)))
        )
    )
    (:durative-action battery_charge_1
        :parameters (?r - robot)
        :duration (= ?duration 1)
        :condition (and
            (at start(charger_approached ?r))
        )
        :effect (and
            (at end(increase (battery_level ?r) 1))
        )
    )
;====================================================
;====================================================
)