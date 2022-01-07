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
    (Pose ?object ?pose)
    (Grasp ?object ?grasp_pose) ; gripper_from_object

    (RackPose ?object ?pose)
    (ClampPose ?clamp ?pose)
    (ScrewDriverPose ?sd ?pose)
    ;; (JointPose ?e1 ?e2 ?pose)
    (ElementGoalPose ?element ?pose)

    ; * static predicates but will be sampled by stream functions
    (PickBeamWithGripperAction ?element ?gripper ?action)
    ;; (BeamPlacementWithClampsAction ?element ?gripper ?action)

    ; * for construction sequence
    (Order ?element1 ?element2)

    ; * Fluent predicates (predicates that change over time, which describes the state of the sytem)
    (Attached ?object ?grasp) ; o can be element, gripper or clamp
    (RobotToolChangerEmpty)
    (RobotGripperEmpty)
    (AtPose ?object ?pose)

    (ElementRackOccupied)
    (NeedGripperRetraction)
    (NeedScrewDriverRetraction)

    (ToolNotOccupiedOnJoint ?tool)
    (ToolAtJoint ?tool ?element1 ?element2 ?adhered_element)
    (JointOccupiedByTool ?element1 ?element2 ?adhered_element)

    ; TODO only certified once, once true always true
    ;; (ToolAssignedToJoint ?element1 ?element2 ?tool)

    (AtRack ?object) ; object can be either element or tool
    (Assembled ?element)

    ;; * derived
    (JointMade ?e1 ?e2)
    (AllScrewDriversNotOccupied)

    (ExistNoClampAtOneAssembledJoints ?element)
    (ExistNoScrewDriverAtOneAssembledJoints ?element)
  )

  (:functions
    (Cost)
  )

  (:action pick_beam_with_gripper
    :parameters (?element ?e_grasp ?tool ?tool_grasp ?action)
    :precondition (and
                    (AllScrewDriversNotOccupied)
                    (ElementRackOccupied)
                    (Gripper ?tool)
                    (GripperToolTypeMatch ?element ?tool)
                    (Attached ?tool ?tool_grasp)
                    (RobotGripperEmpty)
                    (Element ?element)
                    (AtRack ?element)
                    (Grasp ?element ?e_grasp)
                    ; ! e2 must be assembled before e encoded in the given partial ordering
                    (forall (?ei) (imply (Order ?ei ?element) (Assembled ?ei)))
                    ; ! tool state precondition, might help pruning
                    ;; (not (ExistNoClampAtOneAssembledJoints ?element))
                    ; ! sampled
                    (PickBeamWithGripperAction ?element ?gripper ?action)
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
                    (forall (?ei) (imply (Order ?ei ?element) (Assembled ?ei)))
                    ; ! sampled
                    ;; (BeamPlacementWithClampsAction ?element ?tool ?action)
                  )
    :effect (and (Assembled ?element)
                 (NeedGripperRetraction)
                 (AtPose ?element ?e_pose)
            )
  )

  (:action beam_placement_without_clamp
    :parameters (?element ?e_pose ?e_grasp ?tool ?tool_grasp)
    :precondition (and
                    (Gripper ?tool)
                    (Attached ?tool ?tool_grasp)
                    (Attached ?element ?e_grasp)
                    (GroundContactElement ?element)
                    (ElementGoalPose ?element ?e_pose)
                    ; ! e2 must be assembled before e encoded in the given partial ordering
                    (forall (?ei) (imply (Order ?ei ?element) (Assembled ?ei)))
                    ; ! sampled
                    )
    :effect (and (Assembled ?element)
                 (NeedGripperRetraction)
                 (AtPose ?element ?e_pose)
                 )
  )

  ; TODO haven't model screwdrivers that act like grippers yet!
  (:action assemble_beam_with_screwdrivers
    :parameters (?element ?tool)
    :precondition (and
                    (Gripper ?tool)
                    (Attached ?tool)
                    (Attached ?element)
                    (ScrewedWithGripperElement ?element)
                    (ElementGoalPose ?element ?e_pose)
                    ; ! tool state
                    (not (ExistNoScrewDriverAtOneAssembledJoints ?element))
                    ; ! e2 must be assembled before e encoded in the given partial ordering
                    (forall (?ei) (imply (Order ?ei ?element) (Assembled ?ei)))
                    ; ! sampled
                    )
    :effect (and (Assembled ?element)
                 (NeedGripperRetraction)
                 )
  )

  (:action retract_gripper_from_beam
    :parameters (?element ?e_grasp ?tool ?tool_grasp)
    :precondition (and
                    (Gripper ?tool)
                    (Attached ?tool ?tool_grasp)
                    (Attached ?element ?e_grasp)
                    (Assembled ?element)
                    (NeedGripperRetraction)
                    (forall (?scaffold) (imply (AssociatedScaffold ?element ?scaffold) (Assembled ?scaffold)))
                  )
    :effect (and (not (NeedGripperRetraction))
                 (not (Attached ?element ?e_grasp))
                 (RobotGripperEmpty)
            )
  )

  (:action operator_load_beam
    :parameters (?element ?e_pose)
    :precondition (and
                    (not (Assembled ?element))
                    (Element ?element)
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
    :parameters (?element)
    :precondition (and
                    (Scaffold ?element)
                    ; ! e2 must be assembled before e encoded in the given partial ordering
                    (forall (?ei) (imply (Order ?ei ?element) (Assembled ?ei)))
                    )
    :effect (and (Assembled ?element)
                 )
  )

  ; TODO haven't model screwdrivers that act like grippers yet!
  (:action operator_attach_screwdriver
    :parameters (?tool ?element1 ?element2)
    :precondition (and
                    (Attached ?element2)
                    (Assembled ?element1)
                    (ScrewDriver ?tool)
                    (AtRack ?tool)
                    (JointToolTypeMatch ?element1 ?element2 ?tool)
                    (ToolNotOccupiedOnJoint ?tool)
                    (not (JointOccupiedByTool ?element1 ?element2 ?element2))
                    ; ! switch for cutting down meaningless clamp placements
                    (not (JointMade ?element1 ?element2))
                    ; ! assembly state precondition
                    ; ! sampled
                    )
    :effect (and
                 ; ! tool status
                 (ToolAtJoint ?tool ?element1 ?element2 ?element2)
                 (JointOccupiedByTool ?element1 ?element2 ?element2)
                 (not (ToolNotOccupiedOnJoint ?tool))
                 (not (AtRack ?tool))
                 (ToolAssignedToJoint ?element1 ?element2 ?tool)
            )
  )

  (:action pick_tool_from_rack
    :parameters (?tool)
    :precondition (and
                    (RobotToolChangerEmpty)
                    (Tool ?tool)
                    (AtRack ?tool)
                    ; ! sampled
                  )
    :effect (and (Attached ?tool)
                 (not (RobotToolChangerEmpty))
                 ; ! tool status
                 (not (AtRack ?tool))
                 (when (Gripper ?tool) (RobotGripperEmpty))
            )
  )

  (:action place_tool_at_rack
    :parameters (?tool)
    :precondition (and
                    (Attached ?tool)
                    (Tool ?tool)
                    (imply (Gripper ?tool) (RobotGripperEmpty))
                    ; ! sampled
                  )
    :effect (and (not (Attached ?tool))
                 (RobotToolChangerEmpty)
                 (AtRack ?tool)
                ;;  (when (Clamp ?tool) (increase (total-cost) (Cost)))
                 (increase (total-cost) (Cost))
            )
  )

  ;; tool is attached to the robot
  ;; a tool (gripper, clamp) can be placed if the goal place is clear of collision
  (:action place_clamp_to_structure
    :parameters (?tool ?element1 ?element2)
    :precondition (and
                    (Attached ?tool)
                    (Clamp ?tool)
                    (Joint ?element1 ?element2)
                    (ToolNotOccupiedOnJoint ?tool)
                    (JointToolTypeMatch ?element1 ?element2 ?tool)
                    ;; ? (or (Assembled ?element1) (Assembled ?element2))
                    ;; ! (EitherAssembled ?element1 ?element2)
                    (Assembled ?element1)
                    (not (JointOccupiedByTool ?element1 ?element2 ?element1))
                    ; ! switch for cutting down meaningless clamp placements
                    (not (JointMade ?element1 ?element2))
                    ; ! assembly state precondition
                    ; ! sampled
                    )
    :effect (and (not (Attached ?tool))
                 (RobotToolChangerEmpty)
                 ; ! tool status
                 (ToolAtJoint ?tool ?element1 ?element2 ?element1)
                 (JointOccupiedByTool ?element1 ?element2 ?element1)
                 (not (ToolNotOccupiedOnJoint ?tool))
                 (ToolAssignedToJoint ?element1 ?element2 ?tool)
                 )
  )

  (:action pick_clamp_from_structure
    :parameters (?tool ?element1 ?element2)
    :precondition (and
                    (AllScrewDriversNotOccupied)
                    (RobotToolChangerEmpty)
                    (Clamp ?tool)
                    (ToolAtJoint ?tool ?element1 ?element2 ?element1)
                    (JointOccupiedByTool ?element1 ?element2 ?element1)
                    (JointMade ?element1 ?element2)
                    ; ! sampled
                  )
    :effect (and (Attached ?tool)
                 (not (RobotToolChangerEmpty))
                 ; ! tool status
                 (ToolNotOccupiedOnJoint ?tool)
                 (not (ToolAtJoint ?tool ?element1 ?element2 ?element1))
                 (not (JointOccupiedByTool ?element1 ?element2 ?element1))
            )
  )

  (:action dock_with_screwdriver
    :parameters (?tool ?element1 ?element2)
    :precondition (and
                    (RobotToolChangerEmpty)
                    (ScrewDriver ?tool)
                    (ToolAtJoint ?tool ?element1 ?element2 ?element2)
                    (JointOccupiedByTool ?element1 ?element2 ?element2)
                    (JointMade ?element1 ?element2)
                    ; ! sampled
                  )
    :effect (and
                 (NeedScrewDriverRetraction)
                 (not (RobotToolChangerEmpty))
            )
  )

  (:action retract_screwdriver_from_beam
    :parameters (?tool ?element1 ?element2)
    :precondition (and
                    (NeedScrewDriverRetraction)
                    (ScrewDriver ?tool)
                    (ToolAtJoint ?tool ?element1 ?element2 ?element2)
                    (JointOccupiedByTool ?element1 ?element2 ?element2)
                    (JointMade ?element1 ?element2)
                    ; ! sampled
                  )
    :effect (and
                 (not (NeedScrewDriverRetraction))
                 (Attached ?tool)
                 ; ! tool status
                 (ToolNotOccupiedOnJoint ?tool)
                 (not (ToolAtJoint ?tool ?element1 ?element2 ?element2))
                 (not (JointOccupiedByTool ?element1 ?element2 ?element2))
            )
  )

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

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

  (:derived (JointMade ?e1 ?e2)
    (imply (Joint ?e1 ?e2)
           (and (Assembled ?e1) (Assembled ?e2))
    )
  )

  (:derived (AllScrewDriversNotOccupied)
       (forall (?tool) (imply (ScrewDriver ?tool)
                              (ToolNotOccupiedOnJoint ?tool)
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
       (or (ScrewedWithoutGripperElement ?element) (ScrewedWithGripperElement ?element))
       (exists (?ei) (and (Joint ?ei ?element) (Assembled ?ei)
                          (not (JointOccupiedByTool ?ei ?element ?element))
                     )
       )
    )
  )
)


  ; ! workaround for a bug in the adaptive algorithm
  ;; (:derived (ClampOrScrewDriver ?tool)
  ;;     (or (Clamp ?tool) (ScrewDriver ?tool))
  ;; )

