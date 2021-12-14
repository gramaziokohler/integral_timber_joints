(define (domain itj_clamp_only)
  (:requirements :strips :equality)
  (:predicates
    ; switch for including move action or not
    (ConsiderTransition)

    ; * Static predicates (predicates that do not change over time)
    (Element ?element)
    (Joint ?element1 ?element2)
    (IsElement ?element)
    (Grounded ?element)
    (JointToolTypeMatch ?element1 ?element2 ?tool)
    (GripperToolTypeMatch ?element ?tool)

    (Gripper ?g)
    (Clamp ?c)
    (IsGripper ?tool)
    (IsClamp ?tool)
    (IsTool ?tool)

    ; * static predicates but will be produced by stream functions
    (Pose ?pose)
    (Traj ?traj)
    (PlaceTraj ?traj)

    (RobotConf ?conf)
    (PlaceStartRobotConf ?conf)
    (PlaceEndRobotConf ?conf)

    ; * generic pick and place actions for both grippers and clamps for now
    (PlaceToolAction ?object ?conf1 ?conf2 ?traj)
    (PickToolAction ?object ?conf1 ?conf2 ?traj)

    (PlaceElementAction ?object ?conf1 ?conf2 ?traj)
    (PickElementAction ?object ?conf1 ?conf2 ?traj)
    (MoveAction ?conf1 ?conf2 ?traj)

    ; * optional
    (Order ?element1 ?element2)

    ; * Fluent predicates (predicates that change over time, which describes the state of the sytem)
    ;; (ObjectAtPose ?object ?pose)
    ;; (RobotFlangeAtPose ?robot ?pose)

    (RobotAtConf ?conf)
    (Attached ?object) ; o can be element, gripper or clamp
    (RobotToolChangerEmpty)
    (RobotGripperEmpty)
    (CanFreeMove)

    ;; (ToolAtPose ?tool ?pose)
    (ToolAtJoint ?tool ?element1 ?element2)
    (ToolNotOccupiedOnJoint ?tool)
    (NoToolAtJoint ?element1 ?element2)
    ;; (ToolAtConf ?tool ?conf)

    (AtRack ?object) ; object can be either element or tool
    (Assembled ?element)

    ;; * derived
    (Connected ?element)
    (AllToolAtJoints ?element)
    ;; (ExistNoToolAtJoints ?element)
    (EitherGroundedAllToolAtJoints ?element)
  )

  ; ? with or without attached objects share the same `move` action?
  (:action move
    :parameters (?conf1 ?conf2 ?traj)
    :precondition (and
                    ; ! state precondition
                    (RobotAtConf ?conf1)
                    (CanFreeMove)
                    ; ! sampled
                    (MoveAction ?conf1 ?conf2 ?traj)
                )
    :effect (and
                 ; ! state change
                 (not (RobotAtConf ?conf1))
                 (RobotAtConf ?conf2)
                 ; ! switch to avoid transit forever
                 (not (CanFreeMove))
                 ; ! cost-aware
                ;;  (increase (total-cost) (Distance ?conf1 ?conf2))
            )
  )

  (:action pick_element_from_rack
    :parameters (?element ?conf1 ?conf2 ?traj ?tool)
    :precondition (and
                    ; ! state precondition
                    (imply (ConsiderTransition) (and (not (CanFreeMove)) (RobotAtConf ?conf1)))
                    (IsGripper ?tool)
                    (GripperToolTypeMatch ?element ?tool)
                    (Attached ?tool)
                    (RobotGripperEmpty)
                    ; ! sampled
                    (PickElementAction ?element ?conf1 ?conf2 ?traj)
                    (AtRack ?element)
                  )
    :effect (and (not (AtRack ?element))
                 (Attached ?element)
                 (not (RobotGripperEmpty))
                 ; ! robot conf
                 (not (RobotAtConf ?conf1))
                 (RobotAtConf ?conf2)
                 ; ! switch for move
                 (CanFreeMove)
            )
  )

  ; an element can only be placed if all the clamps are (attached) to the corresponding joints
  ; we can query a partial structure and a new element's connection joints using fluent
  (:action place_element_on_structure
    :parameters (?element ?conf1 ?conf2 ?traj ?tool)
    :precondition (and
                    ; ! robot state precondition
                    (imply (ConsiderTransition) (and (not (CanFreeMove)) (RobotAtConf ?conf1)))
                    (IsGripper ?tool)
                    (Attached ?tool)
                    (Attached ?element)
                    ; ! assembly state precondition
                    (Connected ?element)
                    ; ! tool state precondition
                    ;; (or (Grounded ?element) (AllToolAtJoints ?element))
                    (EitherGroundedAllToolAtJoints ?element)
                    ; ! e2 must be assembled before e encoded in the given partial ordering
                    (forall (?ei) (imply (Order ?ei ?element) (Assembled ?ei)))
                    ; ! sampled
                    (PlaceElementAction ?element ?conf1 ?conf2 ?traj)
                    )
    :effect (and (Assembled ?element)
                 (not (Attached ?element))
                 (RobotGripperEmpty)
                 (not (RobotAtConf ?conf1))
                 (RobotAtConf ?conf2)
                 ; ! switch for move
                 (CanFreeMove)
                 )
  )

  (:action pick_gripper_from_rack
    :parameters (?tool ?conf1 ?conf2 ?traj)
    :precondition (and
                    ; ! state precondition
                    (imply (ConsiderTransition) (and (not (CanFreeMove)) (RobotAtConf ?conf1)))
                    (RobotToolChangerEmpty)
                    (IsGripper ?tool)
                    (AtRack ?tool)
                    ; ! sampled
                    (PickToolAction ?tool ?conf1 ?conf2 ?traj)
                  )
    :effect (and (Attached ?tool)
                 (not (RobotToolChangerEmpty))
                 ; ! tool status
                 (not (AtRack ?tool))
                 (RobotGripperEmpty)
                 ; ! robot conf
                 (not (RobotAtConf ?conf1))
                 (RobotAtConf ?conf2)
                 ; ! switch for move
                 (CanFreeMove)
            )
  )

  (:action pick_clamp_from_rack
    :parameters (?tool ?conf1 ?conf2 ?traj)
    :precondition (and
                    ; ! state precondition
                    (imply (ConsiderTransition) (and (not (CanFreeMove)) (RobotAtConf ?conf1)))
                    (RobotToolChangerEmpty)
                    (IsClamp ?tool)
                    (AtRack ?tool)
                    ; ! sampled
                    (PickToolAction ?tool ?conf1 ?conf2 ?traj)
                  )
    :effect (and (Attached ?tool)
                 (not (RobotToolChangerEmpty))
                 (not (AtRack ?tool))
                 ; ! robot conf
                 (not (RobotAtConf ?conf1))
                 (RobotAtConf ?conf2)
                 ; ! switch for move
                 (CanFreeMove)
            )
  )

  (:action place_tool_at_rack
    :parameters (?tool ?conf1 ?conf2 ?traj)
    :precondition (and
                    ; ! robot state precondition
                    (imply (ConsiderTransition) (and (not (CanFreeMove)) (RobotAtConf ?conf1)))
                    (Attached ?tool)
                    (IsTool ?tool)
                    (imply (IsGripper ?tool) (RobotGripperEmpty))
                    ; ! sampled
                    (PlaceToolAction ?tool ?conf1 ?conf2 ?traj)
                    )
    :effect (and (not (Attached ?tool))
                 (RobotToolChangerEmpty)
                 ; ! tool status
                 (AtRack ?tool)
                 ; ! robot conf
                 (not (RobotAtConf ?conf1))
                 (RobotAtConf ?conf2)
                 ; ! switch for move
                 (CanFreeMove)
                 )
  )

  ;; * Specific pick_from_joint action for clamps only
  (:action pick_clamp_from_joint
    :parameters (?tool ?element1 ?element2 ?conf1 ?conf2 ?traj)
    :precondition (and
                    ; ! state precondition
                    (imply (ConsiderTransition) (and (not (CanFreeMove)) (RobotAtConf ?conf1)))
                    (RobotToolChangerEmpty)
                    (IsClamp ?tool)
                    (ToolAtJoint ?tool ?element1 ?element2)
                    ; ! sampled
                    (PickToolAction ?tool ?conf1 ?conf2 ?traj)
                  )
    :effect (and (Attached ?tool)
                 (not (RobotToolChangerEmpty))
                 ; ! tool status
                 (ToolNotOccupiedOnJoint ?tool)
                 (not (ToolAtJoint ?tool ?element1 ?element2))
                 (NoToolAtJoint ?element1 ?element2)
                 ; ! robot conf
                 (not (RobotAtConf ?conf1))
                 (RobotAtConf ?conf2)
                 ; ! switch for move
                 (CanFreeMove)
            )
  )

  ;; tool is attached to the robot
  ;; a tool (gripper, clamp) can be placed if the goal place is clear of collision
  (:action place_clamp_at_joint
    :parameters (?tool ?element1 ?element2 ?conf1 ?conf2 ?traj)
    :precondition (and
                    ; ! robot state precondition
                    (imply (ConsiderTransition) (and (not (CanFreeMove)) (RobotAtConf ?conf1)))
                    (Attached ?tool)
                    (IsClamp ?tool)
                    (Joint ?element1 ?element2)
                    (ToolNotOccupiedOnJoint ?tool)
                    (JointToolTypeMatch ?element1 ?element2 ?tool)
                    (or (Assembled ?element1) (Assembled ?element2))
                    (NoToolAtJoint ?element1 ?element2)
                    ; ! assembly state precondition
                    ; ! sampled
                    (PlaceToolAction ?tool ?conf1 ?conf2 ?traj)
                    )
    :effect (and (not (Attached ?tool))
                 (RobotToolChangerEmpty)
                 ; ! tool status
                 (ToolAtJoint ?tool ?element1 ?element2)
                 (not (NoToolAtJoint ?element1 ?element2))
                 (not (ToolNotOccupiedOnJoint ?tool))
                 ; ! robot conf
                 (not (RobotAtConf ?conf1))
                 (RobotAtConf ?conf2)
                 ; ! switch for move
                 (CanFreeMove)
                 )
  )

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  (:derived (Connected ?element)
   (or (Grounded ?element)
       (exists (?ei) (and (Joint ?ei ?element)
                          (Assembled ?ei)
                          (Connected ?ei)
                     )
       )
   )
  )

;;   (:derived (ExistNoToolAtJoints ?element)
;;        (exists (?ei) (and (Joint ?ei ?element)
;;                           (NoToolAtJoint ?ei ?element)
;;                      )
;;        )
;;   )

  (:derived (AllToolAtJoints ?element)
   (forall (?ei) (imply (Joint ?ei ?element)
                        (not (NoToolAtJoint ?ei ?element))
                        ;; (exists (?tool)
                        ;;     (and (IsClamp ?tool) (Joint ?ei ?element)
                        ;;          (ToolAtJoint ?tool ?ei ?element)
                        ;;          )
                        ;; )
                 )
   )
  )

  (:derived (EitherGroundedAllToolAtJoints ?element)
    (and
        (or (Grounded ?element) (AllToolAtJoints ?element))
        ;; (or (Grounded ?element) (not (ExistNoToolAtJoints ?element)))
    )
  )

)
