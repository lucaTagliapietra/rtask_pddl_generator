(define
  (domain assembly)
  (:requirements :adl)
  (:types
    resource assembly - object
  )
  (:predicates
    (available ?x - object)
    (complete ?a - assembly)
    (requires ?a - assembly ?r - resource)
    (committed ?r - resource ?a - assembly)
    (incorporated ?part - assembly ?whole - assembly)
    (part-of ?part - assembly ?whole - assembly)
    (to-be-removed ?part - assembly ?whole - assembly)
    (assemble-order ?part1 - assembly ?part2 - assembly ?whole - assembly)
    (transient-part ?part - assembly ?whole - assembly)
    (remove-order ?part1 - assembly ?part2 - assembly ?whole - assembly)
  )
  (:action commit
    :parameters (?res - resource ?as - assembly)
    :precondition
      (available ?res)
    :effect
      (and
        (not
          (available ?res)
        )
        (committed ?res ?as)
      )
  )
  (:action release
    :parameters (?res - resource ?as - assembly)
    :precondition
      (committed ?res ?as)
    :effect
      (and
        (not
          (committed ?res ?as)
        )
        (available ?res)
      )
  )
  (:action assemble
    :parameters (?part - assembly ?whole - assembly)
    :precondition
      (and
        (forall (?res - resource)
          (imply
            (requires ?whole ?res)
            (committed ?res ?whole)
          )
        )
        (or
          (part-of ?part ?whole)
          (transient-part ?part ?whole)
        )
        (available ?part)
        (forall (?prev - assembly)
          (imply
            (assemble-order ?prev ?part ?whole)
            (incorporated ?prev ?whole)
          )
        )
      )
    :effect
      (and
        (incorporated ?part ?whole)
        (not
          (available ?part)
        )
        (when
          (and
            (not
              (exists (?p - assembly)
                (and
                  (part-of ?p ?whole)
                  (not
                    (= ?p ?part)
                  )
                  (not
                    (incorporated ?p ?whole)
                  )
                )
              )
            )
            (not
              (exists (?tp - assembly)
                (and
                  (transient-part ?tp ?whole)
                  (incorporated ?tp ?whole)
                  (not
                    (= ?tp ?part)
                  )
                  (not
                    (incorporated ?tp ?whole)
                  )
                )
              )
            )
          )
          (and
            (complete ?whole)
            (available ?whole)
          )
        )
      )
  )
  (:action remove
    :parameters (?part - assembly ?whole - assembly)
    :precondition
      (and
        (forall (?res - resource)
          (imply
            (requires ?whole ?res)
            (committed ?res ?whole)
          )
        )
        (incorporated ?part ?whole)
        (or
          (and
            (transient-part ?part ?whole)
            (forall (?prev1 - assembly)
              (imply
                (remove-order ?prev1 ?part ?whole)
                (incorporated ?prev1 ?whole)
              )
            )
            (and
              (part-of ?part ?whole)
              (not
                (exists (?prev2 - assembly)
                  (and
                    (assemble-order ?prev2 ?part ?whole)
                    (incorporated ?prev2 ?whole)
                  )
                )
              )
            )
          )
        )
      )
    :effect
      (and
        (not
          (incorporated ?part ?whole)
        )
        (available ?part)
        (when
          (and
            (not
              (exists (?p - assembly)
                (and
                  (part-of ?p ?whole)
                  (not
                    (incorporated ?p ?whole)
                  )
                )
              )
            )
            (not
              (exists (?tp - assembly)
                (and
                  (transient-part ?tp ?whole)
                  (not
                    (= ?tp ?part)
                  )
                  (incorporated ?tp ?whole)
                )
              )
            )
          )
          (and
            (complete ?whole)
            (available ?whole)
          )
        )
      )
  )
)
