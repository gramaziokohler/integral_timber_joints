(define (stream itj_clamp_only)

  ; ! do I have to put all preconditions from the action here?
  ; or it's computed "on-demand" from the action?
;;   (:stream sample-pick_beam_with_gripper
;;     ; beam_id, gripper_id
;;     :inputs (?element ?gripper)
;;     :domain (and (Element ?element) (Gripper ?gripper))
;;     :fluents (AtPose Attached)
;;     :outputs (?action)
;;     :certified (and
;;                  (PickBeamWithGripperAction ?element ?gripper ?action)
;;                )
;;   )

;;   (:stream sample-beam_placement_with_clamps
;;     ; beam_id, gripper_id
;;     :inputs (?element ?gripper)
;;     :domain (and (Element ?element) (Gripper ?gripper))
;;     :fluents (AtPose Attached ToolAssignedToJoint)
;;     :outputs (?action)
;;     :certified (and
;;                  (PickBeamWithGripperAction ?element ?gripper ?action)
;;                )
;;   )

;;   (:stream sample-beam_placement_without_clamp
;;     ; beam_id, gripper_id
;;     :inputs (?element ?gripper)
;;     :domain (and (Element ?element) (Gripper ?gripper))
;;     :fluents (AtPose Attached)
;;     :outputs (?action)
;;     :certified (and
;;                  (BeamPlacementWithoutClampsAction ?element ?gripper ?action)
;;                )
;;   )

;;   (:stream sample-assemble_beam_with_screwdrivers_and_gripper_at_rack
;;     ; beam_id, gripper_id
;;     :inputs (?element ?gripper)
;;     :domain (and (Element ?element) (Gripper ?gripper))
;;     :fluents (AtPose Attached)
;;     :outputs (?action)
;;     :certified (and
;;                 (BundledAssemblePlacementWithScrewDriversAction ?element ?gripper ?action)
;;                )
;;   )

;;   (:stream sample-retract_gripper_from_beam
;;     ; beam_id, gripper_id
;;     :inputs (?element ?gripper)
;;     :domain (and (Element ?element) (Gripper ?gripper))
;;     :fluents (AtPose Attached)
;;     :outputs (?action)
;;     :certified (and
;;                  (RetractGripperFromBeamAction ?element ?gripper ?action)
;;                )
;;   )

;;   ;; pick and place tools
;;   (:stream sample-pick_gripper_from_rack
;;     ; beam_id, gripper_id
;;     :inputs (?element ?tool)
;;     :domain (and (Element ?element) (Gripper ?tool))
;;     :fluents (AtPose Attached)
;;     :outputs (?action)
;;     :certified (and
;;                  (PickGripperFromStorageAction ?tool ?action)
;;                )
;;   )

;;   (:stream sample-pick_clamp_from_rack
;;     :inputs (?element ?tool)
;;     :domain (and (Element ?element) (Clamp ?tool))
;;     :fluents (AtPose Attached)
;;     :outputs (?action)
;;     :certified (and
;;                 (PickClampFromStorageAction ?tool ?action)
;;                )
;;   )

;;   (:stream sample-pick_screwdriver_from_rack
;;     :inputs (?element ?tool)
;;     :domain (and (Element ?element) (ScrewDriver ?tool))
;;     :fluents (AtPose Attached)
;;     :outputs (?action)
;;     :certified (and
;;                 (PickScrewdriverFromStorageAction ?tool ?action)
;;                )
;;   )

;;   (:stream sample-place_gripper_at_rack
;;     ; beam_id, gripper_id
;;     :inputs (?element ?tool)
;;     :domain (and (Element ?element) (Gripper ?tool))
;;     :fluents (AtPose Attached)
;;     :outputs (?action)
;;     :certified (and
;;                  (PlaceGripperToStorageAction ?tool ?action)
;;                )
;;   )

;;   (:stream sample-place_clamp_at_rack
;;     :inputs (?element ?tool)
;;     :domain (and (Element ?element) (Clamp ?tool))
;;     :fluents (AtPose Attached)
;;     :outputs (?action)
;;     :certified (and
;;                 (PlaceClampToStorageAction ?tool ?action)
;;                )
;;   )

;;   (:stream sample-place_screwdriver_at_rack
;;     :inputs (?element ?tool)
;;     :domain (and (Element ?element) (ScrewDriver ?tool))
;;     :fluents (AtPose Attached)
;;     :outputs (?action)
;;     :certified (and
;;                 (PlaceScrewdriverToStorageAction ?tool ?action)
;;                )
;;   )

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
