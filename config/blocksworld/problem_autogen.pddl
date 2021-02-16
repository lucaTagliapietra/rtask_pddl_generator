(define
  (problem blocks-4)
  (:domain blocks)
  (:objects
    B C A D - block
  )
  (:init
    (clear C)
    (clear A)
    (clear B)
    (clear D)
    (on-table C)
    (on-table A)
    (on-table B)
    (on-table D)
    (handempty)
  )
  (:goal
    (and
      (on ?D ?C)
      (on ?C ?B)
      (on ?B ?A)
    )
  )
)
