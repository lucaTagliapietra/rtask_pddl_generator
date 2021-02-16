(define
  (domain blocks)
  (:requirements :strips :typing)
  (:predicates
    (on-table ?x - block)
    (clear ?x - block)
    (on ?x - block ?y - block)
    (handempty)
    (holding ?x - block)
  )
  (:action pick-up
    :parameters (?x - block)
    :precondition
      (and
        (clear ?x)
        (on-table ?x)
        (handempty)
      )
    :effect
      (and
        (not
          (on-table ?x)
        )
        (not
          (clear ?x)
        )
        (not
          (handempty)
        )
        (holding ?x)
      )
  )
  (:action put-down
    :parameters (?x - block)
    :precondition
      (holding ?x)
    :effect
      (and
        (not
          (holding ?x)
        )
        (clear ?x)
        (handempty)
        (on-table ?x)
      )
  )
  (:action stack
    :parameters (?x - block ?y - block)
    :precondition
      (and
        (holding ?x)
        (clear ?y)
      )
    :effect
      (and
        (not
          (holding ?x)
        )
        (not
          (clear ?y)
        )
        (clear ?x)
        (handempty)
        (on ?x ?y)
      )
  )
  (:action unstack
    :parameters (?x - block ?y - block)
    :precondition
      (and
        (on ?x ?y)
        (clear ?y)
        (handempty)
      )
    :effect
      (and
        (holding ?x)
        (clear ?y)
        (not
          (clear ?x)
        )
        (not
          (handempty)
        )
        (not
          (on ?x ?y)
        )
      )
  )
)
