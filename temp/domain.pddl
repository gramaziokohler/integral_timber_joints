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

    ; * static predicates but will be produced by stream functions
    (Traj ?traj)
    (PlaceElementAction ?element ?traj)

    ; * for construction sequence
    (Order ?element1 ?element2)

    ; * Fluent predicates (predicates that change over time, which describes the state of the sytem)
    (Attached ?object) ; o can be element, gripper or clamp
    (RobotToolChangerEmpty)
    (RobotGripperEmpty)

    (ElementRackOccupied)
    (NeedGripperRetraction)
    (NeedScrewDriverRetraction)

    (ToolNotOccupiedOnJoint ?tool)
    (ToolAtJoint ?tool ?element1 ?element2 ?adhered_element)
    (JointOccupiedByTool ?element1 ?element2 ?adhered_element)

    ; only certified once
    (ToolAssignedToJoint ?element1 ?element2 ?tool)

    (AtRack ?object) ; object can be either element or tool
    (Assembled ?element)

    ;; * derived
    (Connected ?element)
    (JointMade ?e1 ?e2)

    ;; (ClampOrScrewDriver ?tool)
    (AllScrewDriversNotOccupied)

    (ExistNoClampAtOneAssembledJoints ?element)
    (ExistNoScrewDriverAtOneAssembledJoints ?element)
  )

  (:functions
    (Cost)
  )

  (:action pick_beam_with_gripper
    :parameters (?element ?tool)
    :precondition (and
                    (AllScrewDriversNotOccupied)
                    (ElementRackOccupied)
                    (Gripper ?tool)
                    (GripperToolTypeMatch ?element ?tool)
                    (Attached ?tool)
                    (RobotGripperEmpty)
                    (Element ?element)
                    (AtRack ?element)
                    ; ! assembly state precondition
                    (Connected ?element)
                    ; ! e2 must be assembled before e encoded in the given partial ordering
                    (forall (?ei) (imply (Order ?ei ?element) (Assembled ?ei)))
                    ; ! tool state precondition
                    ;; (EitherGrounedOrExistToolAtJoints ?element)
                    ; ! sampled
                  )
    :effect (and (not (AtRack ?element))
                 (Attached ?element)
                 (not (RobotGripperEmpty))
                 (not (ElementRackOccupied))
            )
  )

  ; an element can only be placed if all the clamps are (attached) to the corresponding joints
  ; we can query a partial structure and a new element's connection joints using fluent
  (:action beam_placement_with_clamps
    :parameters (?element ?traj ?tool)
    :precondition (and
                    (Gripper ?tool)
                    (Attached ?tool)
                    (Attached ?element)
                    (ClampedElement ?element)
                    ; ! assembly state precondition
                    (Connected ?element)
                    ; ! tool state precondition
                    ; ? all joints with assembled elements have tools occipied
                    (not (ExistNoClampAtOneAssembledJoints ?element))
                    ; ! e2 must be assembled before e encoded in the given partial ordering
                    (forall (?ei) (imply (Order ?ei ?element) (Assembled ?ei)))
                    ; ! sampled
                    (PlaceElementAction ?element ?traj)
                    )
    :effect (and (Assembled ?element)
                 (NeedGripperRetraction)
            )
  )

  (:action beam_placement_without_clamp
    :parameters (?element ?traj ?tool)
    :precondition (and
                    (Gripper ?tool)
                    (Attached ?tool)
                    (Attached ?element)
                    (GroundContactElement ?element)
                    ; ! assembly state precondition
                    ;; (Connected ?element)
                    ; ! e2 must be assembled before e encoded in the given partial ordering
                    (forall (?ei) (imply (Order ?ei ?element) (Assembled ?ei)))
                    ; ! sampled
                    (PlaceElementAction ?element ?traj)
                    )
    :effect (and (Assembled ?element)
                 (NeedGripperRetraction)
                 )
  )

  ; TODO haven't model screwdrivers that act like grippers yet!
  (:action assemble_beam_with_screwdrivers
    :parameters (?element ?traj ?tool)
    :precondition (and
                    (Gripper ?tool)
                    (Attached ?tool)
                    (Attached ?element)
                    (ScrewedWithGripperElement ?element)
                    ; ! tool state
                    (not (ExistNoScrewDriverAtOneAssembledJoints ?element))
                    ; ! assembly state precondition
                    (Connected ?element)
                    ; ! e2 must be assembled before e encoded in the given partial ordering
                    (forall (?ei) (imply (Order ?ei ?element) (Assembled ?ei)))
                    ; ! sampled
                    (PlaceElementAction ?element ?traj)
                    )
    :effect (and (Assembled ?element)
                 (NeedGripperRetraction)
                 )
  )

  (:action retract_gripper_from_beam
    :parameters (?element ?tool)
    :precondition (and 
                    (Gripper ?tool)
                    (Attached ?tool)
                    (Attached ?element)
                    (Assembled ?element)
                    (NeedGripperRetraction)
                    (forall (?scaffold) (imply (AssociatedScaffold ?element ?scaffold) (Assembled ?scaffold)))
                  )
    :effect (and (not (NeedGripperRetraction))
                 (not (Attached ?element))
                 (RobotGripperEmpty)
            )
  )

  ;; TODO (:action RetractScrewdriverFromBeamAction

  (:action operator_load_beam
    :parameters (?element)
    :precondition (and
                    (not (Assembled ?element))
                    (Element ?element)
                    (not (ElementRackOccupied))
                    (RobotGripperEmpty)
                    )
    :effect (and (ElementRackOccupied)
                 (AtRack ?element)
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
    :parameters (?tool ?element1 ?element2) ; ?conf1 ?conf2 ?traj)
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
    :parameters (?tool ?element1 ?element2) ; ?conf1 ?conf2 ?traj)
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
    :parameters (?tool ?element1 ?element2) ; ?conf1 ?conf2 ?traj)
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

  (:derived (Connected ?element)
   (or (GroundContactElement ?element)
       (exists (?ei) (and 
                          (Element ?ei)
                          (Joint ?ei ?element)
                          (Assembled ?ei)
                          (Connected ?ei)
                     )
       )
   )
  )

  (:derived (JointMade ?e1 ?e2)
    (imply (Joint ?e1 ?e2) 
           (and (Assembled ?e1) (Assembled ?e2))
    )
  )

  ; ! workaround for a bug in the adaptive algorithm
  ;; (:derived (ClampOrScrewDriver ?tool)
  ;;     (or (Clamp ?tool) (ScrewDriver ?tool))
  ;; )

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
