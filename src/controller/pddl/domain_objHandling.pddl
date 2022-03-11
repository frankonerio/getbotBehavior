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
            balls door pipe vicitm      - item
    )
    ;====================================================
    ;====================================================

    (:predicates
        ;(ballsDetected ?b - balls ?w - world)

        (balls_approached ?r - robot ?b - balls)
        (balls_grabbed ?r - robot ?b - balls)
        (balls_target_approached ?r - robot ?b - balls)
        (balls_dropped ?r - robot ?b - balls)
        (balls_handeled ?r - robot ?b - balls)

        (charger_approached ?r - robot)
    )   

    (:functions
        (battery_level ?r - robot)

        (detected_balls ?b - balls)
        (at_target_balls ?b - balls)
        (carried_balls ?r - robot)
        
        (at_target_balls_goal ?b - balls)
    )
    
;====================================================
;====================================================
    (:durative-action approach_balls
        :parameters (?w - world ?r - robot ?b - balls)
        :duration (= ?duration 5)
        :condition (and
            ;(at start(ballsDetected ?b ?w))
            (at start(balls_dropped ?r ?b))
            (at start(< (carried_balls ?r) 1))
            (at start(> (battery_level ?r) 15))
        )
        :effect (and
            (at end(balls_approached ?r ?b))
            (at end(not(balls_target_approached ?r ?b)))
            (at end(decrease (battery_level ?r) 5))
            ;;;;;;;;;;;;
            (at end (not (charger_approached ?r)))
        )
    )
;====================================================
    (:durative-action grab_balls
        :parameters (?w - world ?r - robot ?b - balls)
        :duration (= ?duration 5)
        :condition (and
            (at start(balls_approached ?r ?b))
            (at start(< (carried_balls ?r) 1))
            (at start(> (battery_level ?r) 13))
        )
        :effect (and
            (at end(balls_grabbed ?r ?b))
            (at end(not(balls_dropped ?r ?b)))
            (at end(decrease (detected_balls ?b) 1))
            (at start(increase (carried_balls ?r) 1))
            (at end(decrease (battery_level ?r) 3))
        )
    )
;====================================================
    (:durative-action approach_balls_target
        :parameters (?w - world ?r - robot ?b - balls)
        :duration (= ?duration 5)
        :condition (and
            (at start(balls_grabbed ?r ?b))
            (at start(> (carried_balls ?r) 0))
            ;(at start(balls_approached ?r ?b))
            (at start(> (battery_level ?r) 15))
        )
        :effect (and
            (at end(not(balls_approached ?r ?b)))
            (at end(balls_target_approached ?r ?b))
            (at end(decrease (battery_level ?r) 5))
            ;;;;;;;;;;;;;
            (at end (not (charger_approached ?r)))
        )
    )
;====================================================
    (:durative-action drop_balls
        :parameters (?w - world ?r - robot ?b - balls)
        :duration (= ?duration 5)
        :condition (and
            (at start(balls_grabbed ?r ?b))
            (at start(> (carried_balls ?r) 0))
            (at start(balls_target_approached ?r ?b))
            (at start(> (battery_level ?r) 12))
        )
        :effect (and
            (at end(balls_dropped ?r ?b))
            (at end(not(balls_grabbed ?r ?b)))
            (at start(increase (at_target_balls ?b) 1))
            (at start(decrease (carried_balls ?r) 1))
            (at end(decrease (battery_level ?r) 2))
        )
    )
;====================================================
    (:durative-action handle_balls
        :parameters (?w - world ?r - robot ?b - balls)
        :duration (= ?duration 0.001)
        :condition (and
            (at start(balls_dropped ?r ?b))
            (at start(< (carried_balls ?r) 1))
            ;(at start(< (detected_balls ?b) 1))
            (at start(> (at_target_balls ?b) (at_target_balls_goal ?b)))
        )
        :effect (and
            (at end(balls_handeled ?r ?b))
        )
    )
;====================================================
    
    ;====================================================
;====================================================
    (:durative-action charger_approach
        :parameters (?r - robot  ?b - balls)
        :duration (= ?duration 5)
        :condition (and
            (at start(> (battery_level ?r) 9))
        )
        :effect (and
            (at end(charger_approached ?r))
            (at end(decrease (battery_level ?r) 10))

            (at end(not(balls_approached ?r ?b)))
            (at end(not(balls_target_approached ?r ?b)))
        )
    )
    (:durative-action battery_charge_1
        :parameters (?r - robot)
        :duration (= ?duration 1)
        :condition (and
            (at start(charger_approached ?r))
        )
        :effect (and
            (at end(increase (battery_level ?r) 10))
        )
    )
;====================================================
;====================================================
)