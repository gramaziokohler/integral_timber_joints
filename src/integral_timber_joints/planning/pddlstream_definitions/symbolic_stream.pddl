(define (stream construction)
   (:stream sample-move
    :inputs (?conf1 ?conf2 ?traj)
    :domain (and (RobotConf ?conf1)
                 (RobotConf ?conf2)
                 (Traj ?traj)
                 )
    ; :fluents (Assembled)
    :outputs (?traj)
    :certified (and
                  (MoveAction ?conf1 ?conf2 ?traj)
                  (Traj ?traj)
               )
  )

  (:stream sample-pick
    :inputs (?object)
    :domain (or (IsTool ?object) (IsElement ?object))
    ; :fluents (Assembled)
    :outputs (?conf1 ?conf2 ?traj)
    :certified (and
                    (PickAction ?object ?conf1 ?conf2 ?traj)
                    (Conf ?conf1)
                    (Conf ?conf2)
                    (Traj ?traj)
                )
  )

  (:stream sample-place
    :inputs (?object)
    :domain (or (IsTool ?object) (IsElement ?object))
    ; :fluents (Assembled)
    :outputs (?conf1 ?conf2 ?traj)
    :certified (and
                    (PlaceAction ?object ?conf1 ?conf2 ?traj)
                    (Conf ?conf1)
                    (Conf ?conf2)
                    (Traj ?traj)
                )
  )

;;   (:stream test-cfree
;;     :inputs (?robot ?trajectory ?attached_object)
;;     :domain (and (Robot ?r) (Traj ?r ?t) (Element ?e))
;;     :certified (CollisionFree ?r ?t ?e)
;;   )

;   (:stream test-stiffness
; ;    :fluents (Printed)
;    :certified (Stiff)
;   )
)
