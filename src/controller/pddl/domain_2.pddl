;====================================================
;====================================================
;AUTHOR: SALEM ALAIDROOS
;====================================================
;====================================================
(define (domain disaster_drescue_robot)
    (:requirements :typing :negative-preconditions :fluents
    :durative-actions )
    ;====================================================
    ;====================================================

    (:types
        world robot navigation item - object
        floors stairs elevator - navigation
        door pipe vicitm - item
    )
    ;====================================================
    ;====================================================

    (:predicates
        (batterycharged ?r - robot)
        (automode ?r - robot)

        (explorable ?w - world)
        (explored ?w - world)
        (mappable ?w - world)
        (mapped ?w - world)
     )   

    ;====================================================
    ;====================================================
    (:durative-action explore
        :parameters (?w - world ?r - robot)
        :duration (= ?duration 5)
        :condition (and
            (at start(batterycharged ?r))
            (at start(automode ?r))
            (at start(explorable ?w))
            ;(at start(not (explored ?w)))
        )
        :effect (and
            (at end(explored ?w))
        )
    )
    ;====================================================
    (:durative-action map
        :parameters (?w - world ?r - robot)
        :duration ( = ?duration 5)
        :condition (and
            (at start(batterycharged ?r))
            (at start(automode ?r))
            (at start(mappable ?w))
            ;(at start(not (mapped ?w)))
        )
        :effect (and
            (at end(mapped ?w))
        )
    )
    ;====================================================
    ;====================================================
)