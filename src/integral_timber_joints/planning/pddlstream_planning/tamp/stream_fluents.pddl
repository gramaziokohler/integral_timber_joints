(define (stream itj_clamp_only)

  (:stream sample-beam_placement_with_clamps
    :inputs (?gripper ?element)
    :domain (and 
                (Gripper ?gripper)
                (ClampedElement ?element)
                (GripperToolTypeMatch ?element ?gripper)
            )
    :fluents (AtPose Attached Assembled ToolAtJoint)
    :outputs (?action)
    :certified (and
                 (BeamPlacementWithClampsAction ?element ?gripper ?action)
               )
  )

  (:stream sample-assemble_beam_with_screwdrivers_with_gripper
    :inputs (?tool ?element)
    :domain (and 
                (Gripper ?tool)
                (ScrewedWithGripperElement ?element)
                (GripperToolTypeMatch ?element ?tool)
            )
    :fluents (AtPose Attached Assembled)
    :outputs (?action)
    :certified (and
                 (AssembleBeamWithScrewdriversWithGripperAction ?element ?tool ?action)
               )
  )

  (:stream sample-assemble_beam_with_screwdrivers_without_gripper
    :inputs (?tool ?element)
    :domain (and 
                (ScrewDriver ?tool)
                (ScrewedWithoutGripperElement ?element)
                (GripperToolTypeMatch ?element ?tool)
            )
    :fluents (AtPose Attached Assembled)
    :outputs (?action)
    :certified (and
                 (AssembleBeamWithScrewdriversWithoutGripperAction ?element ?tool ?action)
               )
  )

;;   ;;;;;;;;;;;;;;;;;

  (:stream sample-place_clamp_to_structure
    :inputs (?tool ?element1 ?element2)
    :domain (and
            (Clamp ?tool)
            (Joint ?element1 ?element2)
            (JointToolTypeMatch ?element1 ?element2 ?tool)
            (ClampedElement ?element2)
            )
    :fluents (AtPose Attached Assembled)
    :outputs (?action)
    :certified (and
                (PlaceClampToStructureAction ?tool ?element1 ?element2 ?action)
               )
  )

  (:stream sample-pick_clamp_from_structure
    :inputs (?tool ?element1 ?element2)
    :domain (and
            (Clamp ?tool)
            (Joint ?element1 ?element2)
            (JointToolTypeMatch ?element1 ?element2 ?tool)
            (ClampedElement ?element2)
            )
    :fluents (AtPose Attached Assembled)
    :outputs (?action)
    :certified (and
                (PickClampFromStructureAction ?tool ?element1 ?element2 ?action)
               )
  )

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
