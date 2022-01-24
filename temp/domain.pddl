(define (domain itj_clamp_only)
  (:requirements :strips :equality)
  (:predicates
    ; * Static predicates (predicates that do not change over time)
    (Element ?element)
    (GroundContactElement ?element)
    (Scaffold ?element) ;; ManualAssembly
    (ClampedElement ?element)
    (ScrewedWithGripperElement ?element)
    (ScrewedWithoutGripperElement ?element)

    (Joint ?element1 ?element2)
    (JointToolTypeMatch ?element1 ?element2 ?tool)
    (GripperToolTypeMatch ?element ?tool)

    (AssociatedScaffold ?element ?scaffold)

    (Gripper ?g)
    (Clamp ?c)
    (ScrewDriver ?sd)
    (Tool ?tool)

    ; * pose and tags
    (Grasp ?object ?grasp_pose) ; toolchanger_from_object
    (GraspViaBeam ?object ?element ?grasp_pose) ; toolchanger_from_object

    (RackPose ?object ?pose)
    (ClampPose ?clamp ?e1 ?e2 ?pose) ; mounted on e1, e1 assembled
    (ScrewDriverPose ?sd ?e1 ?e2 ?pose) ; mounted on e2, e1 assembled
    (ElementGoalPose ?element ?pose)

    ; ? static predicates but will be sampled by stream functions
    (PickBeamWithGripperAction ?element ?gripper ?action)
    ;; (BeamPlacementWithClampsAction ?element ?gripper ?action)
    ;; (BeamPlacementWithoutClampsAction ?element ?gripper ?action)
    ;; (BundledAssemblePlacementWithScrewDriversAction ?element ?gripper ?action)
    ;; (RetractGripperFromBeamAction ?element ?gripper ?action)
    ;
    ;; (PickGripperFromStorageAction ?tool ?action)
    ;; (PickClampFromStorageAction ?tool ?action)
    ;; (PickScrewdriverFromStorageAction ?tool ?action)
    ;; (PlaceGripperToStorageAction ?tool ?action)
    ;; (PlaceClampToStorageAction ?tool ?action)
    ;; (PlaceScrewdriverToStorageAction ?tool ?action)
    ;; (PlaceClampToStructureAction ?tool ?element1 ?element2 ?action)
    ;; (PickClampFromStructureAction ?tool ?element1 ?element2 ?action)

    ; * for construction sequence
    (FirstElement ?element)
    (Order ?element1 ?element2)

    ; * Fluent predicates (predicates that change over time, which describes the state of the sytem)
    (Attached ?object ?grasp) ; o can be element, gripper or clamp
    (RobotToolChangerEmpty)
    (RobotGripperEmpty)
    (AtPose ?object ?pose)

    (ElementRackOccupied)
    (NeedGripperRetraction)

    (ToolNotOccupiedOnJoint ?tool)
    (ToolAtJoint ?tool ?element1 ?element2 ?adhered_element)
    (JointOccupiedByTool ?element1 ?element2 ?adhered_element)

    ; TODO only certified once, once true always true
    (ToolAssignedToJoint ?element1 ?element2 ?tool)

    (AtRack ?object) ; object can be either element or tool
    (Assembled ?element)

    ;; * derived
    (ExistScaffoldNotAssembled ?element)
    (PrevAssembled ?element)

    (ExistNoClampAtOneAssembledJoints ?element)
    ;; (ExistNoScrewDriverAtOneAssembledJoints ?element)
  )

  (:functions
    (Cost)
  )

  (:action pick_beam_with_gripper
    :parameters (?element ?e_grasp ?tool ?tool_grasp ?action)
    :precondition (and
                    (ElementRackOccupied)
                    (Gripper ?tool)
                    (GripperToolTypeMatch ?element ?tool)
                    (Attached ?tool ?tool_grasp)
                    (RobotGripperEmpty)
                    (Element ?element)
                    (AtRack ?element)
                    (Grasp ?element ?e_grasp)
                    ; ! e2 must be assembled before e encoded in the given partial ordering
                    (not (Assembled ?element))
                    (PrevAssembled ?element)
                    ; ! tool state precondition, might help pruning
                    ;; (not (ExistNoClampAtOneAssembledJoints ?element))
                    ; ! sampled
                    (PickBeamWithGripperAction ?element ?tool ?action)
                  )
    :effect (and (not (AtRack ?element))
                 (Attached ?element ?e_grasp)
                 (not (RobotGripperEmpty))
                 (not (ElementRackOccupied))
                 (increase (total-cost) 1)
            )
  )

  ; an element can only be placed if all the clamps are (attached) to the corresponding joints
  ; we can query a partial structure and a new element's connection joints using fluent
  (:action beam_placement_with_clamps
    :parameters (?element ?e_pose ?e_grasp ?tool ?tool_grasp) ; ?action)
    :precondition (and
                    (Gripper ?tool)
                    (Attached ?tool ?tool_grasp)
                    (Attached ?element ?e_grasp)
                    (ClampedElement ?element)
                    (ElementGoalPose ?element ?e_pose)
                    (PrevAssembled ?element)
                    ; ! tool state precondition
                    (not (ExistNoClampAtOneAssembledJoints ?element))
                    ; ! sampled
                    ;; (BeamPlacementWithClampsAction ?element ?gripper ?action)
                  )
    :effect (and
                 (NeedGripperRetraction)
                 (increase (total-cost) 1)
            )
  )

  (:action beam_placement_without_clamp
    :parameters (?element ?e_pose ?e_grasp ?tool ?tool_grasp)
    :precondition (and
                    (Gripper ?tool)
                    ;; (Grasp ?tool ?tool_grasp)
                    ;; (Grasp ?element ?e_grasp)
                    (Attached ?tool ?tool_grasp)
                    (Attached ?element ?e_grasp)
                    (GroundContactElement ?element)
                    (ElementGoalPose ?element ?e_pose)
                    (PrevAssembled ?element)
                    ; ! sampled
                    ;; (BeamPlacementWithoutClampsAction ?element ?gripper ?action)
                    )
    :effect (and
                 (NeedGripperRetraction)
                 (increase (total-cost) 1)
            )
  )

  ; TODO haven't model screwdrivers that act like grippers yet!
  ; ! packing all the screwdriver loading, unloading and return to rack here
  (:action assemble_beam_with_screwdrivers_and_gripper_at_rack
    :parameters (?element ?e_pose ?e_grasp ?tool ?tool_pose ?tool_grasp)
    :precondition (and
                    (Gripper ?tool)
                    (Attached ?tool ?tool_grasp)
                    (Attached ?element ?e_grasp)
                    (ScrewedWithGripperElement ?element)
                    (ElementGoalPose ?element ?e_pose)
                    (RackPose ?tool ?tool_pose)
                    (PrevAssembled ?element)
                    ; ! sampled
                    ;; (BundledAssemblePlacementWithScrewDriversAction ?element ?gripper ?action)
                    )
    :effect (and
                (Assembled ?element)
                (RobotGripperEmpty)
                (not (Attached ?element ?e_grasp))
                ;
                (not (Attached ?tool ?tool_grasp))
                (RobotToolChangerEmpty)
                (AtRack ?tool)
                (AtPose ?tool ?tool_pose)
                (increase (total-cost) 1)
            )
  )

  ; helper action that deals with variable-number of screwdrivers to be "AtPose"d and un-"Attached"
;;   (:action _screw_driver_placed_with_beam
;;     :parameters (?tool ?tool_pose ?tool_grasp ?element1 ?element2)
;;     :precondition (and (ScrewDriver ?tool)
;;                        (ToolAtJoint ?tool ?element1 ?element2 ?element2)
;;                        (Assembled ?element2)
;;                        (ScrewDriverPose ?tool ?element1 ?element2 ?tool_pose)
;;                        (Attached ?tool ?tool_grasp)
;;                        (GraspViaBeam ?tool ?element2 ?tool_grasp)
;;                        (not (OnStructure ?tool))
;;                        (CanAtPoseScrewDriver)
;;                 )
;;     :effect (and
;;                 (AtPose ?tool ?tool_pose)
;;                 (not (Attached ?tool ?tool_grasp))
;;                 (OnStructure ?tool)
;;             )
;;   )

  (:action retract_gripper_from_beam
    :parameters (?element ?e_grasp ?tool ?tool_grasp)
    :precondition (and
                    (NeedGripperRetraction)
                    (Gripper ?tool)
                    (Attached ?tool ?tool_grasp)
                    (Element ?element)
                    (not (ScrewedWithGripperElement ?element))
                    (Attached ?element ?e_grasp)
                    (not (ExistScaffoldNotAssembled ?element))
                    ; ! sampled
                    ;; (RetractGripperFromBeamAction ?element ?gripper ?action)
                  )
    :effect (and
                (not (NeedGripperRetraction))
                (not (Attached ?element ?e_grasp))
                (Assembled ?element)
                (RobotGripperEmpty)
                 (increase (total-cost) 1)
            )
  )

  (:action operator_load_beam
    :parameters (?element ?e_pose)
    :precondition (and
                    (not (NeedGripperRetraction))
                    ; ! this is essential in cutting down search branches!
                    (not (Assembled ?element))
                    (Element ?element)
                    (PrevAssembled ?element)
                    (not (ElementRackOccupied))
                    (RobotGripperEmpty)
                    (RackPose ?element ?e_pose)
                  )
    :effect (and (ElementRackOccupied)
                 (AtRack ?element)
                 (AtPose ?element ?e_pose)
                 (increase (total-cost) 1)
            )
  )

  (:action manaul_assemble_scaffold
    :parameters (?scaffold ?s_pose ?element ?e_grasp)
    :precondition (and
                    (Scaffold ?scaffold)
                    (NeedGripperRetraction)
                    (Attached ?element ?e_grasp)
                    (Element ?element)
                    (ElementGoalPose ?scaffold ?s_pose)
                    (not (Assembled ?scaffold))
                    (AssociatedScaffold ?element ?scaffold)
                  )
    :effect (and (Assembled ?scaffold)
                 (AtPose ?scaffold ?s_pose)
                 (increase (total-cost) 1)
            )
  )

;;   ; TODO haven't model screwdrivers that act like grippers yet!
;;   (:action operator_attach_screwdriver
;;     :parameters (?tool ?tool_pose ?tool_grasp ?element1 ?element2 ?e2_grasp)
;;     :precondition (and
;;                     (ScrewedWithGripperElement ?element2)
;;                     (Assembled ?element1)
;;                     (not (Assembled ?element2))
;;                     (Attached ?element2 ?e2_grasp)
;;                     (ScrewDriver ?tool)
;;                     (AtRack ?tool)
;;                     (RackPose ?tool ?tool_pose)
;;                     (AtPose ?tool ?tool_pose)
;;                     (JointToolTypeMatch ?element1 ?element2 ?tool)
;;                     (ToolNotOccupiedOnJoint ?tool)
;;                     (GraspViaBeam ?tool ?element2 ?tool_grasp)
;;                     (not (JointOccupiedByTool ?element1 ?element2 ?element2))
;;                     ; ! switch for cutting down meaningless clamp placements
;;                     ; ! sampled
;;                   )
;;     :effect (and
;;                  (ToolAtJoint ?tool ?element1 ?element2 ?element2)
;;                 ;;  (Attached ?tool ?tool_grasp)
;;                  (JointOccupiedByTool ?element1 ?element2 ?element2)
;;                  (not (ToolNotOccupiedOnJoint ?tool))
;;                  (not (AtRack ?tool))
;;                  (not (AtPose ?tool ?tool_pose))
;;                 ;;  (ToolAssignedToJoint ?element1 ?element2 ?tool)
;;             )
;;   )

  (:action pick_tool_from_rack
    :parameters (?tool ?pose ?grasp)
    :precondition (and
                    (Grasp ?tool ?grasp)
                    (RobotToolChangerEmpty)
                    (Tool ?tool)
                    (AtRack ?tool)
                    (AtPose ?tool ?pose)
                    ; ! sampled
                    ;; (when (Gripper ?tool) (PickGripperFromStorageAction ?tool ?action))
                    ;; (when (Clamp ?tool) (PickClampFromStorageAction ?tool ?action))
                    ;; (when (ScrewDriver ?tool) (PickScrewdriverFromStorageAction ?tool ?action))
                  )
    :effect (and (Attached ?tool ?grasp)
                 (not (RobotToolChangerEmpty))
                 (not (AtRack ?tool))
                 (not (AtPose ?tool ?pose))
                 (when (Gripper ?tool) (RobotGripperEmpty))
                 (increase (total-cost) 1)
            )
  )

  (:action place_tool_at_rack
    :parameters (?tool ?pose ?grasp)
    :precondition (and
                    (Grasp ?tool ?grasp)
                    (Attached ?tool ?grasp)
                    (Tool ?tool)
                    (imply (Gripper ?tool) (RobotGripperEmpty))
                    (RackPose ?tool ?pose)
                    ; ! sampled
                    ;; (when (Gripper ?tool) (PlaceGripperToStorageAction ?tool ?action))
                    ;; (when (Clamp ?tool) (PlaceClampToStorageAction ?tool ?action))
                    ;; (when (ScrewDriver ?tool) (PlaceScrewdriverToStorageAction ?tool ?action))
                  )
    :effect (and (not (Attached ?tool ?grasp))
                 (RobotToolChangerEmpty)
                 (AtRack ?tool)
                 (AtPose ?tool ?pose)
                 (increase (total-cost) (Cost))
            )
  )

  (:action place_clamp_to_structure
    :parameters (?tool ?pose ?grasp ?element1 ?element2)
    :precondition (and
                    (Clamp ?tool)
                    (Attached ?tool ?grasp)
                    (ClampedElement ?element2)
                    (ClampPose ?tool ?element1 ?element2 ?pose)
                    (Joint ?element1 ?element2)
                    (JointToolTypeMatch ?element1 ?element2 ?tool)
                    (ToolNotOccupiedOnJoint ?tool)
                    (Assembled ?element1)
                    (not (JointOccupiedByTool ?element1 ?element2 ?element1))
                    ; ! switch for cutting down meaningless clamp placements
                    (not (Assembled ?element2))
                    ; ! sampled
                    ;; (PlaceClampToStructureAction ?tool ?element1 ?element2 ?action)
                    )
    :effect (and (not (Attached ?tool ?grasp))
                 (AtPose ?tool ?pose)
                 (RobotToolChangerEmpty)
                 ; ! tool status
                 (ToolAtJoint ?tool ?element1 ?element2 ?element1)
                 (JointOccupiedByTool ?element1 ?element2 ?element1)
                 (not (ToolNotOccupiedOnJoint ?tool))
                 (ToolAssignedToJoint ?element1 ?element2 ?tool)
                 (increase (total-cost) 1)
                 )
  )

  (:action pick_clamp_from_structure
    :parameters (?tool ?pose ?grasp ?element1 ?element2)
    :precondition (and
                    (RobotToolChangerEmpty)
                    (Clamp ?tool)
                    (ClampPose ?tool ?element1 ?element2 ?pose)
                    (AtPose ?tool ?pose)
                    (Grasp ?tool ?grasp)
                    (ToolAtJoint ?tool ?element1 ?element2 ?element1)
                    (JointOccupiedByTool ?element1 ?element2 ?element1)
                    (ClampedElement ?element2)
                    (Assembled ?element1)
                    (Assembled ?element2)
                    ; ! sampled
                    ;; (PickClampFromStructureAction ?tool ?element1 ?element2 ?action)
                  )
    :effect (and (Attached ?tool ?grasp)
                 (not (AtPose ?tool ?pose))
                 (not (RobotToolChangerEmpty))
                 ; ! tool status
                 (ToolNotOccupiedOnJoint ?tool)
                 (not (ToolAtJoint ?tool ?element1 ?element2 ?element1))
                 (not (JointOccupiedByTool ?element1 ?element2 ?element1))
                 (increase (total-cost) 1)
            )
  )

;;   (:action dock_and_retract_screwdriver_from_beam
;;     :parameters (?tool ?grasp ?grasp_via_beam ?element1 ?element2)
;;     :precondition (and
;;                     (RobotToolChangerEmpty)
;;                     (ScrewDriver ?tool)
;;                     ;; (ScrewDriverPose ?tool ?element1 ?element2 ?pose)
;;                     ;; (AtPose ?tool ?pose)
;;                     (Grasp ?tool ?grasp)
;;                     ;; (GraspViaBeam ?tool ?element2 ?grasp_via_beam)
;;                     ;
;;                     (ToolAtJoint ?tool ?element1 ?element2 ?element2)
;;                     (JointOccupiedByTool ?element1 ?element2 ?element2)
;;                     (ScrewedWithGripperElement ?element2)
;;                     (Assembled ?element1)
;;                     (Assembled ?element2)
;;                     ;; (OnStructure ?tool)
;;                     ; ! sampled
;;                   )
;;     :effect (and
;;                 ;;  (NeedScrewDriverRetraction)
;;                  (not (RobotToolChangerEmpty))
;;                  (Attached ?tool ?grasp)
;;                 ;;  (not (Attached ?tool ?grasp_via_beam))
;;                 ;;  (not (AtPose ?tool ?pose))
;;                 ;;  (not (OnStructure ?tool))
;;                  ; ! tool status
;;                  (ToolNotOccupiedOnJoint ?tool)
;;                  (not (ToolAtJoint ?tool ?element1 ?element2 ?element2))
;;                  (not (JointOccupiedByTool ?element1 ?element2 ?element2))
;;                  (CanBackToRack)
;;             )
;;   )

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  (:derived (ExistScaffoldNotAssembled ?element)
        ;; (forall (?scaffold) (imply (AssociatedScaffold ?element ?scaffold) (Assembled ?scaffold)))
    (and
        (Element ?element)
        (exists (?scaffold) (and (AssociatedScaffold ?element ?scaffold) (not (Assembled ?scaffold)))
        )
    )
  )

  (:derived (PrevAssembled ?element)
    ;; (forall (?ei) (imply (Order ?ei ?element) (Assembled ?ei)))
    (or (FirstElement ?element)
        (exists (?ei) (and (Order ?ei ?element) (Assembled ?ei)))
    )
  )

  ; * if there is a joint between the current element and an **assembled** element, the clamp must be there
  (:derived (ExistNoClampAtOneAssembledJoints ?element)
    (and
       (ClampedElement ?element)
       (exists (?ei) (and (Joint ?ei ?element) (Assembled ?ei)
                          (not (JointOccupiedByTool ?ei ?element ?ei))
                     )
       )
    )
  )

;;   (:derived (ExistScrewDriverNotAtPose ?e2)
;;     (and
;;         (ScrewedWithGripperElement ?e2)
;;         ;; (Assembled ?e2)
;;         (exists (?sd_tool ?e1)
;;            (and
;;                 (ScrewDriver ?sd_tool)
;;                 (Assembled ?e1)
;;                 (ToolAtJoint ?sd_tool ?e1 ?e2 ?e2)
;;                 (not (OnStructure ?sd_tool))
;;            )
;;         )
;;     )
;;   )

;;   (:derived (ExistScrewDriversOccupied)
;;        (exists (?tool) (and (ScrewDriver ?tool)
;;                             (not (ToolNotOccupiedOnJoint ?tool))
;;                        )
;;        )
;;   )



;;   (:derived (ExistNoScrewDriverAtOneAssembledJoints ?element)
;;     (and
;;     ;;    (or (ScrewedWithoutGripperElement ?element) (ScrewedWithGripperElement ?element))
;;        (ScrewedWithGripperElement ?element)
;;        (exists (?ei) (and (Joint ?ei ?element) (Assembled ?ei)
;;                           (not (JointOccupiedByTool ?ei ?element ?element))
;;                      )
;;        )
;;     )
;;   )

) ; end domain

  ; ! workaround for a bug in the adaptive algorithm
  ;; (:derived (ClampOrScrewDriver ?tool)
  ;;     (or (Clamp ?tool) (ScrewDriver ?tool))
  ;; )

;;   (:derived (Connected ?element)
;;    (or (GroundContactElement ?element)
;;        (exists (?ei) (and
;;                           (Element ?ei)
;;                           (Joint ?ei ?element)
;;                           (Assembled ?ei)
;;                           (Connected ?ei)
;;                      )
;;        )
;;    )
;;   )
