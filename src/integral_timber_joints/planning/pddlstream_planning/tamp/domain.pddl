(define (domain itj_clamp_only)
  (:requirements :strips :equality)
  (:predicates
    ; * Static predicates (predicates that do not change over time)
    (Element ?element)
    (Scaffold ?element) ;; ManualAssembly
    (GroundContactElement ?element)
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
    ; * beam placement actions
    (BeamPlacementWithoutClampsAction ?element ?tool ?action)
    (BeamPlacementWithClampsAction ?element ?tool ?action)
    (AssembleBeamWithScrewdriversWithGripperAction ?element ?tool ?action)
    (AssembleBeamWithScrewdriversWithoutGripperAction ?element ?tool ?action)

    ; * Clamp on/off structure actions
    (PlaceClampToStructureAction ?tool ?element1 ?element2 ?action)
    (PickClampFromStructureAction ?tool ?element1 ?element2 ?action)

    ;; (RetractGripperFromBeamAction ?element ?gripper ?action)

    ; * Storage actions
    ;; (PickBeamWithGripperAction ?element ?gripper ?action)
    ;; (PickGripperFromStorageAction ?tool ?action)
    ;; (PickClampFromStorageAction ?tool ?action)
    ;; (PickScrewdriverFromStorageAction ?tool ?action)
    ;; (PlaceGripperToStorageAction ?tool ?action)
    ;; (PlaceClampToStorageAction ?tool ?action)
    ;; (PlaceScrewdriverToStorageAction ?tool ?action)

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
    (GripperReadyToPickUpBeam)

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
    (ScrewedElement ?element)

    (ExistNoClampAtOneAssembledJoints ?element)
    ;; (ExistNoScrewDriverAtOneAssembledJoints ?element)
  )

  (:functions
    (Cost)
  )

  (:action pick_beam_with_gripper
    :parameters (?element ?e_grasp ?e_pose ?tool ?tool_grasp)
    :precondition (and
                    (ElementRackOccupied)
                    (Gripper ?tool)
                    (GripperToolTypeMatch ?element ?tool)
                    (Attached ?tool ?tool_grasp)
                    (RobotGripperEmpty)
                    ; ! Clamped and GroundContact Element only
                    (Element ?element)
                    (not (ScrewedElement ?element))
                    (AtRack ?element)
                    (RackPose ?element ?e_pose)
                    (AtPose ?element ?e_pose)
                    (Grasp ?element ?e_grasp)
                    ; ! e2 must be assembled before e encoded in the given partial ordering
                    (not (Assembled ?element))
                    (PrevAssembled ?element)
                    ; ! tool state precondition, might help pruning
                    ;; (not (ExistNoClampAtOneAssembledJoints ?element))
                    ; ! sampled
                    ;; (PickBeamWithGripperAction ?element ?tool ?action)
                  )
    :effect (and (not (AtRack ?element))
                 (not (AtPose ?element ?e_pose))
                 (Attached ?element ?e_grasp)
                 (not (RobotGripperEmpty))
                 (not (ElementRackOccupied))
                 (increase (total-cost) 1)
            )
  )

  (:action generic_gripper_approach_beam_pickup
    :parameters (?element ?tool ?tool_grasp)
    :precondition (and
                    ; ! can be gripper or a screwdriver
                    (Tool ?tool)
                    (GripperToolTypeMatch ?element ?tool)
                    (Attached ?tool ?tool_grasp)
                    (RobotGripperEmpty)
                    (ScrewedElement ?element)
                    ; ! e2 must be assembled before e encoded in the given partial ordering
                    (not (Assembled ?element))
                    (PrevAssembled ?element)
                  )
    :effect (and 
                 (GripperReadyToPickUpBeam)
                 (increase (total-cost) 1)
            )
  )

  (:action close_gripper_on_beam
    :parameters (?element ?e_grasp ?e_pose ?tool ?tool_grasp)
    :precondition (and
                    (GripperReadyToPickUpBeam)
                    (ElementRackOccupied)
                    ; ! can be gripper or a screwdriver
                    (Tool ?tool)
                    (GripperToolTypeMatch ?element ?tool)
                    (Attached ?tool ?tool_grasp)
                    (RobotGripperEmpty)
                    (ScrewedElement ?element)
                    (AtRack ?element)
                    (RackPose ?element ?e_pose)
                    (AtPose ?element ?e_pose)
                    (Grasp ?element ?e_grasp)
                    ; ! e2 must be assembled before e encoded in the given partial ordering
                    (not (Assembled ?element))
                    (PrevAssembled ?element)
                  )
    :effect (and (not (AtRack ?element))
                 (not (AtPose ?element ?e_pose))
                 (Attached ?element ?e_grasp)
                 (not (ElementRackOccupied))
                 (not (RobotGripperEmpty))
                 (not (GripperReadyToPickUpBeam))
                 (increase (total-cost) 1)
            )
  )

  ; an element can only be placed if all the clamps are (attached) to the corresponding joints
  ; we can query a partial structure and a new element's connection joints using fluent
  (:action beam_placement_with_clamps
    :parameters (?element ?e_grasp ?tool ?tool_grasp ?action)
    :precondition (and
                    (Gripper ?tool)
                    (Attached ?tool ?tool_grasp)
                    (Attached ?element ?e_grasp)
                    (ClampedElement ?element)
                    (PrevAssembled ?element)
                    ; ! tool state precondition
                    (not (ExistNoClampAtOneAssembledJoints ?element))
                    ; ! sampled
                    (BeamPlacementWithClampsAction ?element ?tool ?action)
                  )
    :effect (and
                 (NeedGripperRetraction)
                 (increase (total-cost) 1)
            )
  )

  (:action beam_placement_without_clamp
    :parameters (?element ?e_grasp ?tool ?tool_grasp ?action)
    :precondition (and
                    (Gripper ?tool)
                    ;; (Grasp ?tool ?tool_grasp)
                    ;; (Grasp ?element ?e_grasp)
                    (Attached ?tool ?tool_grasp)
                    (Attached ?element ?e_grasp)
                    (GroundContactElement ?element)
                    (PrevAssembled ?element)
                    ; ! sampled
                    (BeamPlacementWithoutClampsAction ?element ?tool ?action)
                    )
    :effect (and
                 (NeedGripperRetraction)
                 (increase (total-cost) 1)
            )
  )

  ; ! packing all the screwdriver loading, unloading and return to rack here
  (:action assemble_beam_with_screwdrivers_with_gripper_bundle
    :parameters (?element ?e_pose ?e_grasp ?tool ?tool_pose ?tool_grasp ?action)
    :precondition (and
                    (Gripper ?tool)
                    (Attached ?tool ?tool_grasp)
                    (Attached ?element ?e_grasp)
                    (ScrewedWithGripperElement ?element)
                    (ElementGoalPose ?element ?e_pose)
                    (RackPose ?tool ?tool_pose)
                    (PrevAssembled ?element)
                    ; ! sampled
                    (AssembleBeamWithScrewdriversWithGripperAction ?element ?tool ?action)
                    )
    :effect (and
                (Assembled ?element)
                (AtPose ?element ?e_pose)
                (RobotGripperEmpty)
                (RobotToolChangerEmpty)
                (not (Attached ?element ?e_grasp))
                ;
                (not (Attached ?tool ?tool_grasp))
                (AtRack ?tool)
                (AtPose ?tool ?tool_pose)
                (increase (total-cost) 1)
            )
  )

  ; ! packing all the screwdriver loading, unloading and return to rack here
  (:action assemble_beam_with_screwdrivers_without_gripper_bundle
    :parameters (?element ?e_pose ?e_grasp ?tool ?tool_pose ?tool_grasp ?action)
    :precondition (and
                    (ScrewDriver ?tool)
                    (Attached ?tool ?tool_grasp)
                    (Attached ?element ?e_grasp)
                    (ScrewedWithoutGripperElement ?element)
                    (ElementGoalPose ?element ?e_pose)
                    (RackPose ?tool ?tool_pose)
                    (PrevAssembled ?element)
                    ; ! sampled
                    (AssembleBeamWithScrewdriversWithoutGripperAction ?element ?tool ?action)
                    )
    :effect (and
                (Assembled ?element)
                (AtPose ?element ?e_pose)
                (RobotGripperEmpty)
                (RobotToolChangerEmpty)
                (not (Attached ?element ?e_grasp))
                ;
                (not (Attached ?tool ?tool_grasp))
                (AtRack ?tool)
                (AtPose ?tool ?tool_pose)
                (increase (total-cost) 1)
            )
  )

  (:action retract_gripper_from_beam
    :parameters (?element ?e_grasp ?e_pose ?tool ?tool_grasp)
    :precondition (and
                    (NeedGripperRetraction)
                    (Gripper ?tool)
                    (Attached ?tool ?tool_grasp)
                    (Element ?element)
                    (ElementGoalPose ?element ?e_pose)
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
                (AtPose ?element ?e_pose)
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
                    (imply (ScrewedElement ?element) (GripperReadyToPickUpBeam))
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
                    ;; TODO scaffolding element might be permuted among its immediate scaffolding neighbors!
                  )
    :effect (and (Assembled ?scaffold)
                 (AtPose ?scaffold ?s_pose)
                 (increase (total-cost) 1)
            )
  )

  (:action pick_tool_from_rack
    :parameters (?tool ?pose ?grasp)
    :precondition (and
                    (Grasp ?tool ?grasp)
                    (RobotToolChangerEmpty)
                    (Tool ?tool)
                    (AtRack ?tool)
                    (AtPose ?tool ?pose)
                    ; ! sampled
                    ;; (imply (Gripper ?tool) (PickGripperFromStorageAction ?tool ?action))
                    ;; (imply (Clamp ?tool) (PickClampFromStorageAction ?tool ?action))
                    ;; (imply (ScrewDriver ?tool) (PickScrewdriverFromStorageAction ?tool ?action))
                  )
    :effect (and (Attached ?tool ?grasp)
                 (not (RobotToolChangerEmpty))
                 (not (AtRack ?tool))
                 (not (AtPose ?tool ?pose))
                 (when (Gripper ?tool) (RobotGripperEmpty))
                 (when (ScrewDriver ?tool) (RobotGripperEmpty))
                 (increase (total-cost) 1)
                 ;; ! model gripper of clamp effect
            )
  )

  (:action place_tool_at_rack
    :parameters (?tool ?pose ?grasp)
    :precondition (and
                    (Grasp ?tool ?grasp)
                    (Attached ?tool ?grasp)
                    (Tool ?tool)
                    (imply (Gripper ?tool) (RobotGripperEmpty))
                    (imply (ScrewDriver ?tool) (RobotGripperEmpty))
                    (RackPose ?tool ?pose)
                    ; ! sampled
                    ;; (imply (Gripper ?tool) (PlaceGripperToStorageAction ?tool ?action))
                    ;; (imply (Clamp ?tool) (PlaceClampToStorageAction ?tool ?action))
                    ;; (imply (ScrewDriver ?tool) (PlaceScrewdriverToStorageAction ?tool ?action))
                  )
    :effect (and (not (Attached ?tool ?grasp))
                 (RobotToolChangerEmpty)
                 (AtRack ?tool)
                 (AtPose ?tool ?pose)
                 (increase (total-cost) (Cost))
            )
  )

  (:action place_clamp_to_structure
    :parameters (?tool ?pose ?grasp ?element1 ?element2 ?action)
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
                    (PlaceClampToStructureAction ?tool ?element1 ?element2 ?action)
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
    :parameters (?tool ?pose ?grasp ?element1 ?element2 ?action)
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
                    (PickClampFromStructureAction ?tool ?element1 ?element2 ?action)
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

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  (:derived (ExistScaffoldNotAssembled ?element)
        ;; (forall (?scaffold) (imply (AssociatedScaffold ?element ?scaffold) (Assembled ?scaffold)))
    (and
        (Element ?element)
        (exists (?scaffold) (and (AssociatedScaffold ?element ?scaffold) (not (Assembled ?scaffold)))
        )
    )
  )

  (:derived (ScrewedElement ?element)
    (and
      (Element ?element)
      (or 
          (ScrewedWithGripperElement ?element) 
          (ScrewedWithoutGripperElement ?element) 
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

) ; end domain