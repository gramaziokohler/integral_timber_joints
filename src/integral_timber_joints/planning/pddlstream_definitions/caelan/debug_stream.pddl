(define (stream itj_clamp_only)

  (:stream sample-place-element
    :inputs (?element)
    :domain (IsElement ?element)
    :outputs (?conf1 ?conf2 ?traj)
    :certified (and
                    (PlaceElementAction ?element ?conf1 ?conf2 ?traj)
                    (PlaceStartRobotConf ?conf1)
                    (PlaceEndRobotConf ?conf2)
                    (PlaceTraj ?traj)
                )
  )

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

   (:stream sample-move
    :inputs (?conf1 ?conf2)
    :domain (and (RobotConf ?conf1)
                 (RobotConf ?conf2)
                 )
    ;; :fluents (Assembled)
    :outputs (?traj)
    :certified (and
                  (MoveAction ?conf1 ?conf2 ?traj)
                  (Traj ?traj)
               )
  )

  (:stream sample-pick-element
    :inputs (?element)
    :domain (IsElement ?element)
    ;; :fluents (Assembled)
    :outputs (?conf1 ?conf2 ?traj)
    :certified (and
                    (PickElementAction ?element ?conf1 ?conf2 ?traj)
                    (RobotConf ?conf1)
                    (RobotConf ?conf2)
                    (Traj ?traj)
                )
  )

  (:stream sample-pick-tool
    :inputs (?object)
    :domain (IsTool ?object)
    ;; :fluents (Assembled)
    :outputs (?conf1 ?conf2 ?traj)
    :certified (and
                    (PickToolAction ?object ?conf1 ?conf2 ?traj)
                    (RobotConf ?conf1)
                    (RobotConf ?conf2)
                    (Traj ?traj)
                )
  )

  (:stream sample-place-tool
    :inputs (?object)
    :domain (IsTool ?object)
    ;; :fluents (Assembled)
    :outputs (?conf1 ?conf2 ?traj)
    :certified (and
                    (PlaceToolAction ?object ?conf1 ?conf2 ?traj)
                    (RobotConf ?conf1)
                    (RobotConf ?conf2)
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
