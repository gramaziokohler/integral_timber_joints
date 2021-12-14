(define (stream itj_clamp_only)
;;    (:stream sample-move
;;     :inputs (?conf1 ?conf2)
;;     :domain (and (RobotConf ?conf1)
;;                  (RobotConf ?conf2)
;;                  )
;;     ;; :fluents (Assembled)
;;     :outputs (?traj)
;;     :certified (and
;;                   (MoveAction ?conf1 ?conf2 ?traj)
;;                   (Traj ?traj)
;;                )
;;   )

  (:stream inverse-kinematics
    :inputs (?object ?pose ?grasp)
    :domain (and (Pose ?object ?pose) (Grasp ?object ?grasp))
    :fluents (AtPose Attached)
    :outputs (?conf)
    :certified (and (RobotConf ?conf)
                    (IKSolution ?object ?pose ?grasp ?conf)
                )
  )


    ; sampled an object called `Action1`
    ; (PickElementMovement 'b0' 'g1' Action1)
    ; Action1 = PickBeamWithGripperAction (itj class instance)
  (:stream sample-pick-element
    :inputs (?object ?tool)
    :domain (and
            (IsElement ?object)
            (IsGripper ?tool)
            (GripperToolTypeMatch ?object ?tool)
            )
    :fluents (AtPose Attached)
    :outputs (?action) ; can be any python object
    :certified (and
                    (PickElementMovement ?object ?tool ?action) ; saying that this is a `traj` to pick up `object` with `tool`
                    ;; (PickElementTraj ?traj)
                    ;; (PickElementRobotConf ?conf1)
                    ;; (PickElementRobotConf ?conf2)
                )
  )

;;   (:stream sample-place-element
;;     :inputs (?element)
;;     :domain (IsElement ?element)
;;     ;; :fluents (Assembled)
;;     :outputs (?conf1 ?conf2 ?traj)
;;     :certified (and
;;                     (PlaceElementAction ?element ?conf1 ?conf2 ?traj)
;;                     (RobotConf ?conf1)
;;                     (RobotConf ?conf2)
;;                     (Traj ?traj)
;;                 )
;;   )

;;   (:stream sample-pick-tool
;;     :inputs (?object)
;;     :domain (IsTool ?object)
;;     ;; :fluents (Assembled)
;;     :outputs (?conf1 ?conf2 ?traj)
;;     :certified (and
;;                     (PickToolAction ?object ?conf1 ?conf2 ?traj)
;;                     (RobotConf ?conf1)
;;                     (RobotConf ?conf2)
;;                     (Traj ?traj)
;;                 )
;;   )

;;   (:stream sample-place-tool
;;     :inputs (?object)
;;     :domain (IsTool ?object)
;;     ;; :fluents (Assembled)
;;     :outputs (?conf1 ?conf2 ?traj)
;;     :certified (and
;;                     (PlaceToolAction ?object ?conf1 ?conf2 ?traj)
;;                     (RobotConf ?conf1)
;;                     (RobotConf ?conf2)
;;                     (Traj ?traj)
;;                 )
;;   )

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