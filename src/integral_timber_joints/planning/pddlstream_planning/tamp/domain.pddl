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

    ; * static predicates but will be sampled by stream functions
    (PickBeamWithGripperAction ?element ?gripper ?action)
    ;; (BeamPlacementWithClampsAction ?element ?gripper ?action)

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
    ;; (NeedScrewDriverRetraction)

    (OnStructure ?screwdriver)
    (CanBackToRack)
    (GroundedAssembled)
    (CanAtPoseScrewDriver)

    (ToolNotOccupiedOnJoint ?tool)
    (ToolAtJoint ?tool ?element1 ?element2 ?adhered_element)
    (JointOccupiedByTool ?element1 ?element2 ?adhered_element)

    ; TODO only certified once, once true always true
    ;; (ToolAssignedToJoint ?element1 ?element2 ?tool)

    (AtRack ?object) ; object can be either element or tool
    (Assembled ?element)

    ;; * derived
    ;; (JointMade ?e1 ?e2)
    ; switch to force all screwdrivers to return to rack right after a ScrewedElement is placed
    ;; (AllScrewDriversNotOccupied)
    (ExistScrewDriversOccupied)
    ;; (AllScrewDriverAtPose ?e)
    (ExistScrewDriverNotAtPose ?e)
    (ExistScaffoldNotAssembled ?element)
    (PrevAssembled ?element)

    ; TODO trick to simplify the construction sequence
    ;; (PreviousElementAssembled ?element)

    (ExistNoClampAtOneAssembledJoints ?element)
    (ExistNoScrewDriverAtOneAssembledJoints ?element)
  )

  (:functions
    (Cost)
  )

  (:action pick_beam_with_gripper
    :parameters (?element ?e_grasp ?tool ?tool_grasp ?action)
    :precondition (and
                    (not (ExistScrewDriversOccupied))
                    (ElementRackOccupied)
                    (Gripper ?tool)
                    (GripperToolTypeMatch ?element ?tool)
                    (Attached ?tool ?tool_grasp)
                    (RobotGripperEmpty)
                    (Element ?element)
                    (AtRack ?element)
                    (Grasp ?element ?e_grasp)
                    ; ! e2 must be assembled before e encoded in the given partial ordering
                    ;; (forall (?ei) (imply (Order ?ei ?element) (Assembled ?ei)))
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
                    ; ! tool state precondition
                    ; ? all joints with assembled elements have tools occipied
                    (not (ExistNoClampAtOneAssembledJoints ?element))
                    ; ! e2 must be assembled before e encoded in the given partial ordering
                    ;; (forall (?ei) (imply (Order ?ei ?element) (Assembled ?ei)))
                    (PrevAssembled ?element)
                    ; ! sampled
                    ;; (BeamPlacementWithClampsAction ?element ?tool ?action)
                  )
    :effect (and
                 (NeedGripperRetraction)
                ;;  (Assembled ?element)
                ;;  (AtPose ?element ?e_pose)
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
                    ; ! e2 must be assembled before e encoded in the given partial ordering
                    ;; (forall (?ei) (imply (Order ?ei ?element) (Assembled ?ei)))
                    (PrevAssembled ?element)
                    ; ! sampled
                    )
    :effect (and
                 (NeedGripperRetraction)
                ;;  (Assembled ?element)
                ;;  (AtPose ?element ?e_pose)
            )
  )

  ; TODO haven't model screwdrivers that act like grippers yet!
  (:action assemble_beam_with_screwdrivers
    :parameters (?element ?e_pose ?e_grasp ?tool ?tool_grasp)
    :precondition (and
                    (Gripper ?tool)
                    (Attached ?tool ?tool_grasp)
                    (Attached ?element ?e_grasp)
                    (ScrewedWithGripperElement ?element)
                    (ElementGoalPose ?element ?e_pose)
                    ; ! tool state, equilavent to AllScrewDriverAtAssembledJoints
                    (not (ExistNoScrewDriverAtOneAssembledJoints ?element))
                    ; ! e2 must be assembled before e encoded in the given partial ordering
                    ;; (forall (?ei) (imply (Order ?ei ?element) (Assembled ?ei)))
                    (PrevAssembled ?element)
                    ; ! sampled
                    )
    :effect (and
                 (NeedGripperRetraction)
                 (CanAtPoseScrewDriver)
            )
  )

  ; helper action that deals with variable-number of screwdrivers to be "AtPose"d and un-"Attached"
  (:action _screw_driver_placed_with_beam
    :parameters (?tool ?tool_pose ?tool_grasp ?element1 ?element2)
    :precondition (and (ScrewDriver ?tool)
                       (ToolAtJoint ?tool ?element1 ?element2 ?element2)
                       (Assembled ?element2)
                       (ScrewDriverPose ?tool ?element1 ?element2 ?tool_pose)
                       (Attached ?tool ?tool_grasp)
                       (GraspViaBeam ?tool ?element2 ?tool_grasp)
                       (not (OnStructure ?tool))
                       (CanAtPoseScrewDriver)
                )
    :effect (and
                (AtPose ?tool ?tool_pose)
                (not (Attached ?tool ?tool_grasp))
                (OnStructure ?tool)
            )
  )

  (:action retract_gripper_from_beam
    :parameters (?element ?e_grasp) ;?tool ?tool_grasp)
    :precondition (and
                    (NeedGripperRetraction)
                    ;; (Gripper ?tool)
                    ;; (Attached ?tool ?tool_grasp)
                    (Element ?element)
                    (Attached ?element ?e_grasp)
                    ; ! enforce all screwdrivers to be 'AtPose'd before doing anything else
                    ;; (imply (ScrewedWithGripperElement ?element) (AllScrewDriverAtPose ?element))
                    (imply (ScrewedWithGripperElement ?element) (not (ExistScrewDriverNotAtPose ?element)))
                    ;; (forall (?scaffold) (imply (AssociatedScaffold ?element ?scaffold) (Assembled ?scaffold)))
                    (not (ExistScaffoldNotAssembled ?element))
                  )
    :effect (and
                (not (NeedGripperRetraction))
                (not (Attached ?element ?e_grasp))
                (Assembled ?element)
                (CanBackToRack)
                (RobotGripperEmpty)
                (when (GroundContactElement ?element) (GroundedAssembled))
                (when (ScrewedWithGripperElement ?element) (not (CanAtPoseScrewDriver)))
            )
  )

  (:action operator_load_beam
    :parameters (?element ?e_pose)
    :precondition (and
                    (not (NeedGripperRetraction))
                    ; ! this is essential in cutting down search branches!
                    (not (Assembled ?element))
                    (Element ?element)
                    ;; (forall (?ei) (imply (Order ?ei ?element) (Assembled ?ei)))
                    (PrevAssembled ?element)
                    (not (ElementRackOccupied))
                    (RobotGripperEmpty)
                    (RackPose ?element ?e_pose)
                  )
    :effect (and (ElementRackOccupied)
                 (AtRack ?element)
                 (AtPose ?element ?e_pose)
            )
  )

  (:action manaul_assemble_scaffold
    :parameters (?scaffold ?s_pose ?element ?e_grasp)
    :precondition (and
                    (Scaffold ?scaffold)
                    (NeedGripperRetraction)
                    (Element ?element)
                    (Attached ?element ?e_grasp)
                    (ElementGoalPose ?scaffold ?s_pose)
                    ; ! e2 must be assembled before e encoded in the given partial ordering
                    ;; (forall (?ei) (imply (Order ?ei ?element) (Assembled ?ei)))
                    (not (Assembled ?scaffold))
                    (AssociatedScaffold ?element ?scaffold)
                  )
    :effect (and (Assembled ?scaffold)
                 (AtPose ?scaffold ?s_pose)
            )
  )

  ; TODO haven't model screwdrivers that act like grippers yet!
  (:action operator_attach_screwdriver
    :parameters (?tool ?tool_pose ?tool_grasp ?element1 ?element2 ?e2_grasp)
    :precondition (and
                    (ScrewedWithGripperElement ?element2)
                    (Assembled ?element1)
                    (not (Assembled ?element2))
                    (Attached ?element2 ?e2_grasp)
                    (ScrewDriver ?tool)
                    (AtRack ?tool)
                    (RackPose ?tool ?tool_pose)
                    (AtPose ?tool ?tool_pose)
                    (JointToolTypeMatch ?element1 ?element2 ?tool)
                    (ToolNotOccupiedOnJoint ?tool)
                    (GraspViaBeam ?tool ?element2 ?tool_grasp)
                    (not (JointOccupiedByTool ?element1 ?element2 ?element2))
                    ; ! switch for cutting down meaningless clamp placements
                    ; ! sampled
                  )
    :effect (and
                 (ToolAtJoint ?tool ?element1 ?element2 ?element2)
                 (Attached ?tool ?tool_grasp)
                 (JointOccupiedByTool ?element1 ?element2 ?element2)
                 (not (ToolNotOccupiedOnJoint ?tool))
                 (not (AtRack ?tool))
                 (not (AtPose ?tool ?tool_pose))
                 (not (CanBackToRack))
                ;;  (ToolAssignedToJoint ?element1 ?element2 ?tool)
            )
  )

  (:action pick_gripper_from_rack
    :parameters (?tool ?pose ?grasp)
    :precondition (and
                    (RobotToolChangerEmpty)
                    (Gripper ?tool)
                    (AtRack ?tool)
                    (AtPose ?tool ?pose)
                    (Grasp ?tool ?grasp)
                  )
    :effect (and (Attached ?tool ?grasp)
                 (not (RobotToolChangerEmpty))
                 (RobotGripperEmpty)
                 (not (AtRack ?tool))
                 (not (AtPose ?tool ?pose))
                 (not (CanBackToRack))
            )
  )

  (:action pick_tool_from_rack
    :parameters (?tool ?pose ?grasp)
    :precondition (and
                    (RobotToolChangerEmpty)
                    (Tool ?tool)
                    (not (Gripper ?tool))
                    (AtRack ?tool)
                    (AtPose ?tool ?pose)
                    (Grasp ?tool ?grasp)
                    (GroundedAssembled)
                    ; ! sampled
                  )
    :effect (and (Attached ?tool ?grasp)
                 (not (RobotToolChangerEmpty))
                 (not (AtRack ?tool))
                 (not (AtPose ?tool ?pose))
                 (not (CanBackToRack))
            )
  )

  (:action place_gripper_at_rack
    :parameters (?tool ?pose ?grasp)
    :precondition (and
                    (CanBackToRack)
                    (Grasp ?tool ?grasp)
                    (Attached ?tool ?grasp)
                    (Gripper ?tool)
                    (RackPose ?tool ?pose)
                    (RobotGripperEmpty)
                  )
    :effect (and (not (Attached ?tool ?grasp))
                 (RobotToolChangerEmpty)
                 (AtRack ?tool)
                 (AtPose ?tool ?pose)
                 (increase (total-cost) (Cost))
            )
  )

  (:action place_tool_at_rack
    :parameters (?tool ?pose ?grasp)
    :precondition (and
                    (CanBackToRack)
                    (Grasp ?tool ?grasp)
                    (Attached ?tool ?grasp)
                    (Tool ?tool)
                    (not (Gripper ?tool))
                    (RackPose ?tool ?pose)
                  )
    :effect (and (not (Attached ?tool ?grasp))
                 (RobotToolChangerEmpty)
                 (AtRack ?tool)
                 (AtPose ?tool ?pose)
                 (increase (total-cost) (Cost))
            )
  )

  ;; tool is attached to the robot
  ;; a tool (gripper, clamp) can be placed if the goal place is clear of collision
  (:action place_clamp_to_structure
    :parameters (?tool ?pose ?grasp ?element1 ?element2)
    :precondition (and
                    (Attached ?tool ?grasp)
                    (Clamp ?tool)
                    (ClampedElement ?element2)
                    (ClampPose ?tool ?element1 ?element2 ?pose)
                    (Joint ?element1 ?element2)
                    (JointToolTypeMatch ?element1 ?element2 ?tool)
                    (ToolNotOccupiedOnJoint ?tool)
                    (Assembled ?element1)
                    (not (JointOccupiedByTool ?element1 ?element2 ?element1))
                    ; ! switch for cutting down meaningless clamp placements
                    (not (Assembled ?element2))
                    ; ! assembly state precondition
                    ; ! sampled
                    )
    :effect (and (not (Attached ?tool ?grasp))
                 (AtPose ?tool ?pose)
                 (RobotToolChangerEmpty)
                 ; ! tool status
                 (ToolAtJoint ?tool ?element1 ?element2 ?element1)
                 (JointOccupiedByTool ?element1 ?element2 ?element1)
                 (not (ToolNotOccupiedOnJoint ?tool))
                ;;  (ToolAssignedToJoint ?element1 ?element2 ?tool)
                 )
  )

  (:action pick_clamp_from_structure
    :parameters (?tool ?pose ?grasp ?element1 ?element2)
    :precondition (and
                    ;; (AllScrewDriversNotOccupied)
                    (not (ExistScrewDriversOccupied))
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
                  )
    :effect (and (Attached ?tool ?grasp)
                 (not (AtPose ?tool ?pose))
                 (not (RobotToolChangerEmpty))
                 ; ! tool status
                 (ToolNotOccupiedOnJoint ?tool)
                 (not (ToolAtJoint ?tool ?element1 ?element2 ?element1))
                 (not (JointOccupiedByTool ?element1 ?element2 ?element1))
                 (CanBackToRack)
            )
  )

  (:action dock_and_retract_screwdriver_from_beam
    :parameters (?tool ?grasp ?grasp_via_beam ?element1 ?element2)
    :precondition (and
                    (RobotToolChangerEmpty)
                    (ScrewDriver ?tool)
                    ;; (ScrewDriverPose ?tool ?element1 ?element2 ?pose)
                    ;; (AtPose ?tool ?pose)
                    (Grasp ?tool ?grasp)
                    ;; (GraspViaBeam ?tool ?element2 ?grasp_via_beam)
                    ;
                    (ToolAtJoint ?tool ?element1 ?element2 ?element2)
                    (JointOccupiedByTool ?element1 ?element2 ?element2)
                    (ScrewedWithGripperElement ?element2)
                    (Assembled ?element1)
                    (Assembled ?element2)
                    (OnStructure ?tool)
                    ; ! sampled
                  )
    :effect (and
                ;;  (NeedScrewDriverRetraction)
                 (not (RobotToolChangerEmpty))
                 (Attached ?tool ?grasp)
                ;;  (not (Attached ?tool ?grasp_via_beam))
                ;;  (not (AtPose ?tool ?pose))
                 (not (OnStructure ?tool))
                 ; ! tool status
                 (ToolNotOccupiedOnJoint ?tool)
                 (not (ToolAtJoint ?tool ?element1 ?element2 ?element2))
                 (not (JointOccupiedByTool ?element1 ?element2 ?element2))
                 (CanBackToRack)
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

  (:derived (PrevAssembled ?element)
    ;; (forall (?ei) (imply (Order ?ei ?element) (Assembled ?ei)))
    (or (FirstElement ?element)
        (exists (?ei) (and (Order ?ei ?element) (Assembled ?ei)))
    )
  )

  (:derived (ExistScrewDriverNotAtPose ?e2)
    (and
        (ScrewedWithGripperElement ?e2)
        ;; (Assembled ?e2)
        (exists (?sd_tool ?e1)
           (and
                (ScrewDriver ?sd_tool)
                (Assembled ?e1)
                (ToolAtJoint ?sd_tool ?e1 ?e2 ?e2)
                (not (OnStructure ?sd_tool))
           )
        )
    )
  )

  (:derived (ExistScrewDriversOccupied)
       (exists (?tool) (and (ScrewDriver ?tool)
                            (not (ToolNotOccupiedOnJoint ?tool))
                       )
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

  (:derived (ExistNoScrewDriverAtOneAssembledJoints ?element)
    (and
    ;;    (or (ScrewedWithoutGripperElement ?element) (ScrewedWithGripperElement ?element))
       (ScrewedWithGripperElement ?element)
       (exists (?ei) (and (Joint ?ei ?element) (Assembled ?ei)
                          (not (JointOccupiedByTool ?ei ?element ?element))
                     )
       )
    )
  )

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
