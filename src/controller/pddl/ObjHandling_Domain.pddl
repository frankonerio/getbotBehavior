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
            balls boxes      - item
    )
    ;====================================================
    ;====================================================

    (:predicates
        (items_approached ?r - robot ?i - item)
        (items_grabbed ?r - robot ?i - item)
        (items_target_approached ?r - robot ?i - item)
        (items_dropped ?r - robot ?i - item)
        (items_handeled ?r - robot ?i - item)
    )   

    (:functions
        (battery_level ?r - robot)

        (detected_items ?i - item)
        (at_target_items ?i - item)
        (carried_items ?r - robot)
        
        (at_target_items_goal ?i - item)
    )
    
;====================================================
;====================================================
    (:durative-action approach_items
        :parameters (?w - world ?r - robot ?i - item)
        :duration (= ?duration 5)
        :condition (and
            (at start(items_dropped ?r ?i))
            ;(at start(> (detected_items ?i) 0))
            (at start(< (carried_items ?r) 1))
            (at start(> (battery_level ?r) 5))
        )
        :effect (and
            (at end(items_approached ?r ?i))
            (at end(not(items_target_approached ?r ?i)))
            (at end(decrease (battery_level ?r) 5))
        )
    )
;====================================================
    (:durative-action grab_items
        :parameters (?w - world ?r - robot ?i - item)
        :duration (= ?duration 5)
        :condition (and
            (at start(items_approached ?r ?i))
            (at start(< (carried_items ?r) 1))
            (at start(> (battery_level ?r) 3))
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
        :parameters (?w - world ?r - robot ?i - item)
        :duration (= ?duration 5)
        :condition (and
            (at start(items_grabbed ?r ?i))
            (at start(> (carried_items ?r) 0))
            (at start(items_approached ?r ?i))
            (at start(> (battery_level ?r) 5))
        )
        :effect (and
            (at end(not(items_approached ?r ?i)))
            (at end(items_target_approached ?r ?i))
            (at end(decrease (battery_level ?r) 5))
        )
    )
;====================================================
    (:durative-action drop_items
        :parameters (?w - world ?r - robot ?i - item)
        :duration (= ?duration 5)
        :condition (and
            (at start(items_grabbed ?r ?i))
            (at start(> (carried_items ?r) 0))
            (at start(items_target_approached ?r ?i))
            (at start(> (battery_level ?r) 2))
        )
        :effect (and
            (at end(items_dropped ?r ?i))
            (at end(not(items_grabbed ?r ?i)))
            (at start(increase (at_target_items ?i) 1))
            (at start(decrease (carried_items ?r) 1))
            ;(at start(decrease (detected_items ?i) 1))
            (at end(decrease (battery_level ?r) 2))
        )
    )
;====================================================
    (:durative-action handle_items
        :parameters (?w - world ?r - robot ?i - item)
        :duration (= ?duration 0.001)
        :condition (and
            (at start(items_dropped ?r ?i))
            (at start(< (carried_items ?r) 1))
            (at start(> (at_target_items ?i) (at_target_items_goal ?i)))
        )
        :effect (and
            (at end(items_handeled ?r ?i))
        )
    )
;====================================================
)
