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

    ; pose tags
    (RackPose ?object ?pose)
    (ClampPose ?clamp ?pose)
    (JointPose ?e1 ?e2 ?pose)
    (ElementGoalPose ?element ?pose)

    ; * static predicates but will be produced by stream functions
    (Pose ?object ?pose)

    (Traj ?traj)
    (Grasp ?object ?grasp_pose) ; gripper_from_object

    (RobotConf ?conf)
    ;; (ToolConf ?tool ?conf)

    ; * generic pick and place actions for both grippers and clamps for now
    (IKSolution ?object ?pose ?grasp ?conf)

    ;; (PlaceToolAction ?object ?conf1 ?conf2 ?traj)
    ;; (PickToolAction ?object ?conf1 ?conf2 ?traj)
    ;; (PlaceElementAction ?object ?conf1 ?conf2 ?traj)
    ;; (PickElementAction ?object ?conf1 ?conf2 ?traj)
    ;; (MoveAction ?conf1 ?conf2 ?traj)

    ; * optional
    (Order ?element1 ?element2)

    ; * Fluent predicates (predicates that change over time, which describes the state of the sytem)
    (AtPose ?object ?pose)

    (RobotAtConf ?conf)
    (Attached ?object ?grasp) ; o can be element, gripper or clamp
    (RobotToolChangerEmpty)
    (RobotGripperEmpty)
    (CanFreeMove)

    (ToolAtJoint ?tool ?element1 ?element2)
    (ToolNotOccupiedOnJoint ?tool)
    (NoToolAtJoint ?element1 ?element2)
    ;; (ToolAtConf ?tool ?conf)

    (ElementRackOccupied)

    ;; * derived
    (AtRack ?object) ; object can be either element or tool
    (Assembled ?element)
    (Connected ?element)
    (AllToolAtJoints ?element)
    ;; (EitherAssembled ?element1 ?element2)
  )

  ; ? with or without attached objects share the same `move` action?
;;   (:action move
;;     :parameters (?conf1 ?conf2 ?traj)
;;     :precondition (and
;;                     ; ! state precondition
;;                     (RobotAtConf ?conf1)
;;                     (CanFreeMove)
;;                     ; ! sampled
;;                     (MoveAction ?conf1 ?conf2 ?traj)
;;                 )
;;     :effect (and
;;                  ; ! state change
;;                  (not (RobotAtConf ?conf1))
;;                  (RobotAtConf ?conf2)
;;                  ; ! switch to avoid transit forever
;;                  (not (CanFreeMove))
;;                  ; ! cost-aware
;;                 ;;  (increase (total-cost) (Distance ?conf1 ?conf2))
;;             )
;;   )

  (:action operator_load_element_on_rack
    :parameters (?element ?e_pose)
    :precondition (and
                    (IsElement ?element)
                    (RackPose ?element ?e_pose)
                    (not (ElementRackOccupied))
                  )
    :effect (and
                 (AtPose ?element ?e_pose)
                 (ElementRackOccupied)
            )
  )

  (:action pick_element_from_rack
    :parameters (?element ?e_pose ?e_grasp ?tool ?tool_grasp ?conf)
    :precondition (and
                    ; ! state precondition
                    (imply (ConsiderTransition) (and (not (CanFreeMove)) (RobotAtConf ?conf)))
                    (IsGripper ?tool)
                    (GripperToolTypeMatch ?element ?tool)
                    (Attached ?tool ?tool_grasp)
                    (RobotGripperEmpty)
                    (IsElement ?element)
                    (AtPose ?element ?e_pose)
                    (RackPose ?element ?e_pose)
                    ; ! sampled
                    (IKSolution ?element ?e_pose ?e_grasp ?conf)
                  )
    :effect (and
                 (not (AtPose ?element ?e_pose))
                 (Attached ?element ?e_grasp)
                 (not (RobotGripperEmpty))
                 (not (ElementRackOccupied))
                 ; ! switch for move
                 (CanFreeMove)
            )
  )

  ; an element can only be placed if all the clamps are (attached) to the corresponding joints
  ; we can query a partial structure and a new element's connection joints using fluent
  (:action place_element_on_structure
    :parameters (?element ?e_pose ?e_grasp ?tool ?tool_grasp ?conf)
    :precondition (and
                    ; ! robot state precondition
                    (imply (ConsiderTransition) (and (not (CanFreeMove)) (RobotAtConf ?conf)))
                    (IsGripper ?tool)
                    (Attached ?tool ?tool_grasp)
                    (Attached ?element ?e_grasp)
                    (IsElement ?element)
                    (ElementGoalPose ?element ?e_pose)
                    ; ! assembly state precondition
                    (Connected ?element)
                    ; ! tool state precondition
                    ;; (imply (not (Grounded ?element)) (AllToolAtJoints ?element))
                    ; ! e2 must be assembled before e encoded in the given partial ordering
                    (forall (?ei) (imply (Order ?ei ?element) (Assembled ?ei)))
                    ; ! sampled
                    (IKSolution ?element ?e_pose ?e_grasp ?conf)
                    )
    :effect (and
                 (Assembled ?element)
                 (AtPose ?element ?e_pose)
                 (not (Attached ?element ?e_grasp))
                 (RobotGripperEmpty)
                 ; ! switch for move
                 (CanFreeMove)
                 )
  )

  (:action pick_tool_from_rack
    :parameters (?tool ?pose ?grasp ?conf)
    :precondition (and
                    ; ! state precondition
                    (imply (ConsiderTransition) (and (not (CanFreeMove)) (RobotAtConf ?conf)))
                    (RobotToolChangerEmpty)
                    ;; (IsGripper ?tool)
                    (RackPose ?tool ?pose)
                    (AtPose ?tool ?pose)
                    ; ! sampled
                    (IKSolution ?tool ?pose ?grasp ?conf)
                    ;; (PickToolAction ?tool ?conf1 ?conf2 ?traj)
                  )
    :effect (and (Attached ?tool ?grasp)
                 (not (RobotToolChangerEmpty))
                 ; ! tool status
                 (not (AtPose ?tool ?pose))
                 ; ! for gripper
                 (when (IsGripper ?tool) (RobotGripperEmpty))
                 ; ! switch for move
                 (CanFreeMove)
            )
  )

  (:action place_tool_at_rack
    :parameters (?tool ?pose ?grasp ?conf)
    :precondition (and
                    ; ! robot state precondition
                    (imply (ConsiderTransition) (and (not (CanFreeMove)) (RobotAtConf ?conf)))
                    (IsTool ?tool)
                    (Attached ?tool ?grasp)
                    (imply (IsGripper ?tool) (RobotGripperEmpty))
                    (RackPose ?tool ?pose)
                    ; ! sampled
                    ;; (PlaceToolAction ?tool ?conf1 ?conf2 ?traj)
                    (IKSolution ?tool ?pose ?grasp ?conf)
                    )
    :effect (and (not (Attached ?tool ?grasp))
                 (RobotToolChangerEmpty)
                 ; ! tool status
                 (AtPose ?tool ?pose)
                 ; ! switch for move
                 (CanFreeMove)
                 )
  )

  ;; * Specific pick_from_joint action for clamps only
  (:action pick_clamp_from_joint
    :parameters (?clamp ?pose ?grasp ?element1 ?element2 ?conf)
    :precondition (and
                    ; ! state precondition
                    (imply (ConsiderTransition) (and (not (CanFreeMove)) (RobotAtConf ?conf)))
                    (RobotToolChangerEmpty)
                    (IsClamp ?clamp)
                    (ToolAtJoint ?clamp ?element1 ?element2)
                    (ClampPose ?clamp ?pose)
                    (JointPose ?element1 ?element2 ?pose)
                    (AtPose ?clamp ?pose)
                    ; ! sampled
                    ;; (PickToolAction ?clamp ?conf1 ?conf2 ?traj)
                    (IKSolution ?clamp ?pose ?grasp ?conf)
                  )
    :effect (and (Attached ?clamp ?grasp)
                 (not (RobotToolChangerEmpty))
                 (not (AtPose ?clamp ?pose))
                 ; ! tool status
                 (ToolNotOccupiedOnJoint ?clamp)
                 (not (ToolAtJoint ?clamp ?element1 ?element2))
                 (NoToolAtJoint ?element1 ?element2)
                 ; ! switch for move
                 (CanFreeMove)
            )
  )

  ;; tool is attached to the robot
  ;; a tool (gripper, clamp) can be placed if the goal place is clear of collision
  (:action place_clamp_at_joint
    :parameters (?clamp ?pose ?grasp ?element1 ?element2 ?conf)
    :precondition (and
                    ; ! robot state precondition
                    (imply (ConsiderTransition) (and (not (CanFreeMove)) (RobotAtConf ?conf)))
                    (Attached ?clamp ?grasp)
                    (IsClamp ?clamp)
                    ; ! pose specific to the clamp and joint
                    (ClampPose ?clamp ?pose)
                    (JointPose ?element1 ?element2 ?pose)
                    ; ! tool status
                    (ToolNotOccupiedOnJoint ?clamp)
                    (NoToolAtJoint ?element1 ?element2)
                    (JointToolTypeMatch ?element1 ?element2 ?clamp)
                    ; ! assembly state precondition
                    (or (Assembled ?element1) (Assembled ?element2))
                    ;; (EitherAssembled ?element1 ?element2)
                    ; ! sampled
                    (IKSolution ?clamp ?pose ?grasp ?conf)
                    ;; (PlaceToolAction ?tool ?conf1 ?conf2 ?traj)
                    )
    :effect (and (not (Attached ?clamp ?grasp))
                 (AtPose ?clamp ?pose)
                 (RobotToolChangerEmpty)
                 ; ! tool status
                 (ToolAtJoint ?clamp ?element1 ?element2)
                 (not (NoToolAtJoint ?element1 ?element2))
                 (not (ToolNotOccupiedOnJoint ?clamp))
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

  (:derived (AllToolAtJoints ?element)
   (forall (?ei) (imply (Joint ?element ?ei)
                        (exists (?tool) (ToolAtJoint ?tool ?element ?ei))
                 )
   )
  )

  ; ! workaround for a bug in the adaptive algorithm
;;   (:derived (EitherAssembled ?e1 ?e2)
;;       (and (Joint ?e1 ?e2)
;;         (or (Assembled ?e1) (Assembled ?e2))
;;       )
;;   )

  (:derived (AtRack ?object)
    (exists (?pose) (and (RackPose ?object ?pose)
                         (AtPose ?object ?pose))
                      )
  )

  ; ! pyplanner doesn't like derived predicates in the goal literals
;;   (:derived (Assembled ?element)
;;     (exists (?pose) (and (Element ?element)
;;                       (ElementGoalPose ?element ?pose)
;;                       (AtPose ?element ?pose))
;;                       )
;;   )

)
