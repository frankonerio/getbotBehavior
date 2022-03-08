;====================================================
;====================================================
;AUTHOR: SALEM ALAIDROOS
;====================================================
;====================================================
(define
    ;====================================================
    ;==================================================== 
    (domain DDR_background)
    (:requirements 
        :typing 
        :negative-preconditions
        :fluents
        :durative-actions
    )
    ;====================================================
    ;====================================================

    (:types
        world robot navigation obj - object
    )
    ;====================================================
    ;====================================================

    (:predicates
        (map_available ?r - robot ?w - world)
        (exploring ?r - robot ?w - world)
        (mapping ?r - robot ?w - world)
        (objDetecting ?r - robot ?w - world)

        (charger_approached ?r - robot)

         (obj_approached ?r - robot ?o - obj)
         (item_victim_target_approached ?r - robot ?iv - item_victim)
    )   

    (:functions
        (battery_level ?r - robot)
    )
    
;====================================================
;====================================================
    (:durative-action explore_start
        :parameters (?w - world ?r - robot)
        :duration (= ?duration 1)
        :condition (and
            (at start(map_available ?r ?w))
            (at start(> (battery_level ?r) 10))
        )
        :effect (and
            (at end(exploring ?r ?w))
            ;(at end(decrease (batterylevel ?r) 5))
        )
    )
;====================================================
    (:durative-action map_start
        :parameters (?w - world ?r - robot)
        :duration (= ?duration 1)
        :condition (and
            (at start(exploring ?r ?w))
            (at start(> (battery_level ?r) 10))
        )
        :effect (and
            (at end(mapping ?r ?w))
            (at end(map_available ?r ?w))
            ;(at end(decrease (batterylevel ?r) 5))
        )
    )
;====================================================
    (:durative-action wander_around
        :parameters (?w - world ?r - robot)
        :duration (= ?duration 5)
        :condition (and
            (at start(> (battery_level ?r) 15))
        )
        :effect (and
            (at end(map_available ?r ?w))
            (at end(decrease (battery_level ?r) 5))
        )
    )
;====================================================
    (:durative-action objDetect_start
        :parameters (?w - world ?r - robot)
        :duration (= ?duration 1)
        :condition (and
            (at start(> (battery_level ?r) 10))
        )
        :effect (and
            (at end(objDetecting ?r ?w))
            ;(at end(decrease (batterylevel ?r) 5))
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
            (over all(charger_approached ?r))
        )
        :effect (and
            (at end(increase (battery_level ?r) 1))
        )
    )
;====================================================
;====================================================
)