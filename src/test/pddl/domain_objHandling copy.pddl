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
            (at start(> (battery_level ?r) 5))
        )
        :effect (and
            (at end(balls_approached ?r ?b))
            (at end(not(balls_target_approached ?r ?b)))
            (at end(decrease (battery_level ?r) 5))
        )
    )
;====================================================
    (:durative-action grab_balls
        :parameters (?w - world ?r - robot ?b - balls)
        :duration (= ?duration 5)
        :condition (and
            (at start(balls_approached ?r ?b))
            (at start(< (carried_balls ?r) 1))
            (at start(> (battery_level ?r) 3))
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
            (at start(balls_approached ?r ?b))
            (at start(> (battery_level ?r) 5))
        )
        :effect (and
            (at end(not(balls_approached ?r ?b)))
            (at end(balls_target_approached ?r ?b))
            (at end(decrease (battery_level ?r) 5))
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
            (at start(> (battery_level ?r) 2))
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
        :duration (= ?duration 5)
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
)