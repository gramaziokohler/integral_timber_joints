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
                    ;; (Grasp ?tool ?tool_grasp)
                    ;; (Grasp ?element ?e_grasp)
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
    :parameters (?element ?e_pose ?e_grasp ?tool ?tool_grasp)
    :precondition (and
                    (Gripper ?tool)
                    (Attached ?tool ?tool_grasp)
                    (Attached ?element ?e_grasp)
                    (ScrewedWithGripperElement ?element)
                    (ElementGoalPose ?element ?e_pose)
                    ; ! tool state
                    (not (ExistNoScrewDriverAtOneAssembledJoints ?element))
                    ; ! e2 must be assembled before e encoded in the given partial ordering
                    (forall (?ei) (imply (Order ?ei ?element) (Assembled ?ei)))
                    ; ! sampled
                    )
    :effect (and (Assembled ?element)
                 (AtPose ?element ?e_pose)
                 (NeedGripperRetraction)
                 ;;    (Attached ?sd_tool ?sd_grasp)
                 (forall (?sd_tool ?sd_pose ?e_prev)
                    (when (and
                               (ScrewDriver ?sd_tool)
                               (ToolAtJoint ?sd_tool ?e_prev ?element ?element)
                               (ScrewDriverPose ?sd_tool ?e_prev ?element ?sd_pose)
                          )
                          (and
                               (AtPose ?sd_tool ?sd_pose)
                          )
                    )
                 )
                ;; (GraspViaBeam ?sd_tool ?element ?sd_grasp)
                ;; (not (Attached ?sd_tool ?sd_grasp))
                ;; (ToolAtJoint ?sd_tool ?element1 ?element ?element)
                 ; TODO attached screwdrivers not attached
            )
  )

;;   ; helper action that deals with variable-number of screwdrivers to be "AtPose"d and un-"Attached"
  (:action _screw_driver_placed_with_beam
    :parameters (?tool ?tool_pose ?tool_grasp ?element1 ?element2)
    :precondition (and (ScrewDriver ?tool)
                       (ToolAtJoint ?tool ?element1 ?element2 ?element2)
                       (Assembled ?element2)
                       (ScrewDriverPose ?tool ?element1 ?element2 ?tool_pose)
                       (Attached ?tool ?tool_grasp)
                       (GraspViaBeam ?tool ?element2 ?tool_grasp)
                )
    :effect (and
                (AtPose ?tool ?tool_pose)
                (not (Attached ?tool ?tool_grasp))
            )
  )

  (:action retract_gripper_from_beam
    :parameters (?element ?e_grasp ?tool ?tool_grasp)
    :precondition (and
                    (Gripper ?tool)
                    (Attached ?tool ?tool_grasp)
                    (Attached ?element ?e_grasp)
                    (Assembled ?element)
                    ;; (imply (ScrewedWithGripperElement ?element) (forall (?e) ?element))
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
    :parameters (?element ?e_pose)
    :precondition (and
                    (Scaffold ?element)
                    (ElementGoalPose ?element ?e_pose)
                    ; ! e2 must be assembled before e encoded in the given partial ordering
                    (forall (?ei) (imply (Order ?ei ?element) (Assembled ?ei)))
                  )
    :effect (and (Assembled ?element)
                 (AtPose ?element ?e_pose)
            )
  )

  ; TODO haven't model screwdrivers that act like grippers yet!
  (:action operator_attach_screwdriver
    :parameters (?tool ?tool_pose ?tool_grasp ?element1 ?element2 ?e_grasp)
    :precondition (and
                    (Attached ?element2 ?e_grasp)
                    (Assembled ?element1)
                    (ScrewDriver ?tool)
                    (AtRack ?tool)
                    (RackPose ?tool ?tool_pose)
                    (AtPose ?tool ?tool_pose)
                    (JointToolTypeMatch ?element1 ?element2 ?tool)
                    (ToolNotOccupiedOnJoint ?tool)
                    (GraspViaBeam ?tool ?element2 ?tool_grasp)
                    (not (JointOccupiedByTool ?element1 ?element2 ?element2))
                    ; ! switch for cutting down meaningless clamp placements
                    (not (JointMade ?element1 ?element2))
                    ; ! sampled
                  )
    :effect (and
                 ; ! tool status
                 (ToolAtJoint ?tool ?element1 ?element2 ?element2)
                 (Attached ?tool ?tool_grasp)
                 (JointOccupiedByTool ?element1 ?element2 ?element2)
                 (not (ToolNotOccupiedOnJoint ?tool))
                 (not (AtRack ?tool))
                 (not (AtPose ?tool ?tool_pose))
                ;; TODO screwdriver attached to robot?
                ;;  (ToolAssignedToJoint ?element1 ?element2 ?tool)
            )
  )

  (:action pick_tool_from_rack
    :parameters (?tool ?pose ?grasp)
    :precondition (and
                    (RobotToolChangerEmpty)
                    (Tool ?tool)
                    (AtRack ?tool)
                    (AtPose ?tool ?pose)
                    (Grasp ?tool ?grasp)
                    ; ! sampled
                  )
    :effect (and (Attached ?tool ?grasp)
                 (not (RobotToolChangerEmpty))
                 (when (Gripper ?tool) (RobotGripperEmpty))
                 ; ! tool status
                 (not (AtRack ?tool))
                 (not (AtPose ?tool ?pose))
            )
  )

  (:action place_tool_at_rack
    :parameters (?tool ?pose ?grasp)
    :precondition (and
                    (Grasp ?tool ?grasp)
                    (Attached ?tool ?grasp)
                    (Tool ?tool)
                    (RackPose ?tool ?pose)
                    (imply (Gripper ?tool) (RobotGripperEmpty))
                    ; ! sampled
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
                    (Clamp ?tool)
                    (Attached ?tool ?grasp)
                    (ClampPose ?tool ?element1 ?element2 ?pose)
                    (JointToolTypeMatch ?element1 ?element2 ?tool)
                    (Joint ?element1 ?element2)
                    (ToolNotOccupiedOnJoint ?tool)
                    (Assembled ?element1)
                    (not (JointOccupiedByTool ?element1 ?element2 ?element1))
                    ; ! switch for cutting down meaningless clamp placements
                    (not (JointMade ?element1 ?element2))
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
                    (AllScrewDriversNotOccupied)
                    (RobotToolChangerEmpty)
                    (Clamp ?tool)
                    (ClampPose ?tool ?element1 ?element2 ?pose)
                    (AtPose ?tool ?pose)
                    (Grasp ?tool ?grasp)
                    (ToolAtJoint ?tool ?element1 ?element2 ?element1)
                    (JointOccupiedByTool ?element1 ?element2 ?element1)
                    (JointMade ?element1 ?element2)
                    ; ! sampled
                  )
    :effect (and (Attached ?tool ?grasp)
                 (not (AtPose ?tool ?pose))
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
    :parameters (?tool ?pose ?grasp ?element1 ?element2)
    :precondition (and
                    (NeedScrewDriverRetraction)
                    (ScrewDriver ?tool)
                    (ScrewDriverPose ?tool ?element1 ?element2 ?pose)
                    (AtPose ?tool ?pose)
                    (Grasp ?tool ?grasp)
                    (ToolAtJoint ?tool ?element1 ?element2 ?element2)
                    (JointOccupiedByTool ?element1 ?element2 ?element2)
                    (JointMade ?element1 ?element2)
                    ; ! sampled
                  )
    :effect (and
                 (not (NeedScrewDriverRetraction))
                 (Attached ?tool ?grasp)
                 (not (AtPose ?tool ?pose))
                 ; ! tool status
                 (ToolNotOccupiedOnJoint ?tool)
                 (not (ToolAtJoint ?tool ?element1 ?element2 ?element2))
                 (not (JointOccupiedByTool ?element1 ?element2 ?element2))
            )
  )

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;   (:derived (AllScrewDriverDetached)
;;     (and (ScrewDriver ?tool)
;;          (ToolAtJoint ?tool ?element1 ?element2 ?element2)
;;          (Assembled ?element2)
;;          (ScrewDriverPose ?tool ?element1 ?element2 ?tool_pose)
;;          (AtPose ?tool ?tool_pose)
;;     )
;;   )

  (:derived (JointMade ?e1 ?e2)
        (and (Joint ?e1 ?e2) (Assembled ?e1) (Assembled ?e2))
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
