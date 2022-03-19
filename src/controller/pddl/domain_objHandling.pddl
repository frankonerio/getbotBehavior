;====================================================
;====================================================
;AUTHOR: SALEM ALAIDROOS
;====================================================
;====================================================
(define
    ;====================================================
    ;==================================================== 
    (domain disaster_drescue_robot_)
    (:requirements 
        :typing 
        :negative-preconditions
        :fluents
        :durative-actions
    )
    ;====================================================
    ;====================================================

    (:types
        world robot item     - object
            items door pipe vicitm      - item
    )
    ;====================================================
    ;====================================================

    (:predicates
        ;(itemsDetected ?i - items ?w - world)

        (items_approached ?r - robot ?i - items)
        (items_grabbed ?r - robot ?i - items)
        (items_target_approached ?r - robot ?i - items)
        (items_dropped ?r - robot ?i - items)
        (items_handeled ?r - robot ?i - items)

        (charger_approached ?r - robot)

        ;======================================
    ;Backgorund_Domain
    ;======================================
        (map_available ?r - robot ?w - world)
        (exploring ?r - robot ?w - world)
        (mapping ?r - robot ?w - world)
        (obj_detecting ?r - robot ?w - world)
        
        ;(charger_approached ?r - robot)
    )   

    (:functions
        (battery_level ?r - robot)

        (detected_items ?i - items)
        (at_target_items ?i - items)
        (carried_items ?r - robot)
        
        (at_target_items_goal ?i - items)
    )
    
;====================================================
;====================================================
    (:durative-action approach_items
        :parameters (?w - world ?r - robot ?i - items)
        :duration (= ?duration 5)
        :condition (and
            ;(at start(itemsDetected ?i ?w))
            (at start(items_dropped ?r ?i))
            (at start(< (carried_items ?r) 1))
            (at start(> (battery_level ?r) 15))
        )
        :effect (and
            (at end(items_approached ?r ?i))
            (at end(not(items_target_approached ?r ?i)))
            (at end(decrease (battery_level ?r) 5))
            ;;;;;;;;;;;;
            (at end (not (charger_approached ?r)))
        )
    )
;====================================================
    (:durative-action grab_items
        :parameters (?w - world ?r - robot ?i - items)
        :duration (= ?duration 5)
        :condition (and
            (at start(items_approached ?r ?i))
            (at start(< (carried_items ?r) 1))
            (at start(> (battery_level ?r) 13))
        )
        :effect (and
            (at end(items_grabbed ?r ?i))
            (at end(not(items_dropped ?r ?i)))
            (at end(decrease (detected_items ?i) 1))
            (at start(increase (carried_items ?r) 1))
            (at end(decrease (battery_level ?r) 3))
        )
    )
;====================================================
    (:durative-action approach_items_target
        :parameters (?w - world ?r - robot ?i - items)
        :duration (= ?duration 5)
        :condition (and
            (at start(items_grabbed ?r ?i))
            (at start(> (carried_items ?r) 0))
            ;(at start(items_approached ?r ?i))
            (at start(> (battery_level ?r) 15))
        )
        :effect (and
            (at end(not(items_approached ?r ?i)))
            (at end(items_target_approached ?r ?i))
            (at end(decrease (battery_level ?r) 5))
            ;;;;;;;;;;;;;
            (at end (not (charger_approached ?r)))
        )
    )
;====================================================
    (:durative-action drop_items
        :parameters (?w - world ?r - robot ?i - items)
        :duration (= ?duration 5)
        :condition (and
            (at start(items_grabbed ?r ?i))
            (at start(> (carried_items ?r) 0))
            (at start(items_target_approached ?r ?i))
            (at start(> (battery_level ?r) 12))
        )
        :effect (and
            (at end(items_dropped ?r ?i))
            (at end(not(items_grabbed ?r ?i)))
            (at start(increase (at_target_items ?i) 1))
            (at start(decrease (carried_items ?r) 1))
            (at end(decrease (battery_level ?r) 2))
        )
    )
;====================================================
    (:durative-action handle_items
        :parameters (?w - world ?r - robot ?i - items)
        :duration (= ?duration 0.001)
        :condition (and
            (at start(items_dropped ?r ?i))
            (at start(< (carried_items ?r) 1))
            ;(at start(< (detected_items ?i) 1))
            (at start(> (at_target_items ?i) (at_target_items_goal ?i)))
        )
        :effect (and
            (at end(items_handeled ?r ?i))
        )
    )
;====================================================
    
    ;====================================================
;====================================================
    (:durative-action charger_approach
        :parameters (?r - robot  ?i - items)
        :duration (= ?duration 5)
        :condition (and
            (at start(> (battery_level ?r) 9))
        )
        :effect (and
            (at end(charger_approached ?r))
            (at end(decrease (battery_level ?r) 5))

            (at end(not(items_approached ?r ?i)))
            (at end(not(items_target_approached ?r ?i)))
        )
    )
    (:durative-action battery_charge_1
        :parameters (?r - robot)
        :duration (= ?duration 5)
        :condition (and
            (at start(charger_approached ?r))
        )
        :effect (and
            (at end(increase (battery_level ?r) 30))
        )
    )
;====================================================
;====================================================
    ;======================================
    ;Backgorund_Domain
    ;======================================

    ;==============
    (:durative-action explore_start
        :parameters (?w - world ?r - robot)
        :duration (= ?duration 1)
        :condition (and
            (over all (map_available ?r ?w))
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


    ;======================================
    ;Navigation_Domain
    ;======================================

    ;======================================
    ;ObjHandeling_Domain
    ;======================================

;========================================================================================================
;========================================================================================================

)
