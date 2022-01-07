(define (stream itj_clamp_only)

  (:stream sample-pick_beam_with_gripper
    ; beam_id, gripper_id
    :inputs (?element ?gripper)
    :domain (and (Element ?element) (Gripper ?gripper))
    :fluents (AtPose Attached)
    :outputs (?action)
    :certified (and
                 (PickBeamWithGripperAction ?element ?gripper ?action)
               )
  )

;;   (:stream sample-beam_placement_with_clamps
;;     ; beam_id, joint_ids, gripper_id, clamp_ids
;;     :inputs (?element)
;;     :domain (Element ?element)
;;     :fluents (AtPose Attached)
;;     :outputs (?action)
;;     :certified (and
;;                     (BeamPlacementWithClampsAction ?element ?action)
;;                 )
;;   )

) ; end define

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ; IDEAS
  ; TODO let the user implement this, even if it's deterministic
  ;; (:stream sample-joint-tool
  ;;   :inputs (?e1 ?e2)
  ;;   :domain (Joint ?e1 ?e2)
  ;;   ;; :fluents (Assembled)
  ;;   :outputs (?tool)
  ;;   :certified (and
  ;;                   (Tool ?traj)
  ;;                   (JointToolTypeMatch ?element1 ?element2 ?tool)
  ;;               )
  ;; )
  ; TODO let the user implement this, even if it's deterministic
  ;; (:stream sample-joint-tool
  ;;   :inputs (?e1 ?e2)
  ;;   :domain (Joint ?e1 ?e2)
  ;;   ;; :fluents (Assembled)
  ;;   :outputs (?tool)
  ;;   :certified (and
  ;;                   (Tool ?traj)
  ;;                   (JointToolTypeMatch ?element1 ?element2 ?tool)
  ;;               )
  ;; )
