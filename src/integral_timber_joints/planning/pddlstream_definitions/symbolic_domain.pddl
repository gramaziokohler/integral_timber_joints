(define (domain itj_clamp_only)
  (:requirements :strips :equality)
  (:predicates
    ; * Static predicates (predicates that do not change over time)
    (Element ?element)
    (Joint ?element1 ?element2)
    (IsElement ?element)
    (Grounded ?element)
    ; (JointToolType ?element1 ?element2 ?tool)

    (Gripper ?g)
    (Clamp ?c)
    (IsGripper ?tool)
    (IsClamp ?tool)
    (IsTool ?tool)

    ; * static predicates but will be produced by stream functions
    (Pose ?pose)
    (Traj ?traj)

    (RobotConf ?conf)
    ;; (ToolConf ?tool ?conf)

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
    (CanMove)

    ;; (ToolAtPose ?tool ?pose)
    (ToolAtJoint ?tool ?element1 ?element2)
    (ToolIdle ?tool)
    (NoToolAtJoint ?element1 ?element2)
    ;; (ToolAtConf ?tool ?conf)

    (AtRack ?object) ; object can be either element or tool
    (Assembled ?element)

    ;; * derived
    (Connected ?element)
    (AllToolAtJoints ?element)
  )

  ; ? with or without attached objects share the same `move` action?
;;   (:action move
;;     :parameters (?conf1 ?conf2 ?traj)
;;     :precondition (and
;;                     ; ! state precondition
;;                     (RobotAtConf ?conf1)
;;                     (CanMove)
;;                     ; ! sampled
;;                     (MoveAction ?conf1 ?conf2 ?traj)
;;                 )
;;     :effect (and
;;                  ; ! state change
;;                  (not (RobotAtConf ?conf1))
;;                  (RobotAtConf ?conf2)
;;                  ; ! switch to avoid transit forever
;;                  (not (CanMove))
;;                  ; ! cost-aware
;;                 ;;  (increase (total-cost) (Distance ?conf1 ?conf2))
;;             )
;;   )

  (:action pick_element_from_rack
    :parameters (?element ?conf1 ?conf2 ?traj ?tool)
    :precondition (and
                    ; ! state precondition
                    ;; (RobotAtConf ?conf1)
                    (IsGripper ?tool)
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
                ;;  (not (RobotAtConf ?conf1))
                ;;  (RobotAtConf ?conf2)
                 ; ! switch for move
                 (CanMove)
            )
  )

  ; an element can only be placed if all the clamps are (attached) to the corresponding joints
  ; we can query a partial structure and a new element's connection joints using fluent
  (:action place_element_on_structure
    :parameters (?element ?conf1 ?conf2 ?traj ?tool)
    :precondition (and
                    ; ! robot state precondition
                    ;; (RobotAtConf ?conf1)
                    (IsGripper ?tool)
                    (Attached ?tool)
                    (Attached ?element)
                    ; ! assembly state precondition
                    ;; (Connected ?element)
                    ; ! tool state precondition
                    ;; (AllToolAtJoints ?element)
                    ; ! e2 must be assembled before e encoded in the given partial ordering
                    (forall (?ei) (imply (Order ?ei ?element) (Assembled ?ei)))
                    ; ! sampled
                    (PlaceElementAction ?element ?conf1 ?conf2 ?traj)
                    )
    :effect (and (Assembled ?element)
                 (not (Attached ?element))
                 (RobotGripperEmpty)
                ;;  (not (RobotAtConf ?conf1))
                ;;  (RobotAtConf ?conf2)
                 ; ! switch for move
                 (CanMove)
                 )
  )

  ;; * Generic pick_from_rack action for all types of tools
  (:action pick_tool_from_rack
    :parameters (?tool ?conf1 ?conf2 ?traj)
    :precondition (and
                    ; ! state precondition
                    ;; (RobotAtConf ?conf1)
                    (RobotToolChangerEmpty)
                    (IsTool ?tool)
                    (AtRack ?tool)
                    ; ! sampled
                    (PickToolAction ?tool ?conf1 ?conf2 ?traj)
                  )
    :effect (and (Attached ?tool)
                 (not (RobotToolChangerEmpty))
                 (RobotGripperEmpty) ; ! doesn't hurt to turn on for clamp?
                 ; ! tool status
                ;;  (ToolIdle ?tool)
                 (not (AtRack ?tool))
                 ; ! robot conf
                ;;  (not (RobotAtConf ?conf1))
                ;;  (RobotAtConf ?conf2)
                 ; ! switch for move
                 (CanMove)
            )
  )

  (:action place_tool_at_rack
    :parameters (?tool ?conf1 ?conf2 ?traj)
    :precondition (and
                    ; ! robot state precondition
                    ;; (RobotAtConf ?conf1)
                    (Attached ?tool)
                    (IsTool ?tool)
                    ;; (ToolIdle ?tool)
                    ; ! sampled
                    (PlaceToolAction ?tool ?conf1 ?conf2 ?traj)
                    )
    :effect (and (not (Attached ?tool))
                 (RobotToolChangerEmpty)
                 (not (RobotGripperEmpty)) ; ! doesn't hurt to turn on for clamp?
                 ; ! tool status
                 (AtRack ?tool)
                ;;  (not (ToolIdle ?tool))
                 ; ! robot conf
                ;;  (not (RobotAtConf ?conf1))
                ;;  (RobotAtConf ?conf2)
                 ; ! switch for move
                 (CanMove)
                 )
  )

;;   ;; * Specific pick_from_joint action for clamps only
;;   (:action pick_clamp_from_joint
;;     :parameters (?tool ?element1 ?element2 ?conf1 ?conf2 ?traj)
;;     :precondition (and
;;                     ; ! state precondition
;;                     ;; (RobotAtConf ?conf1)
;;                     (RobotToolChangerEmpty)
;;                     (IsClamp ?tool)
;;                     (ToolAtJoint ?tool ?element1 ?element2)
;;                     ; ! sampled
;;                     (PickToolAction ?tool ?conf1 ?conf2 ?traj)
;;                   )
;;     :effect (and (Attached ?tool)
;;                  (not (RobotToolChangerEmpty))
;;                  ; ! tool status
;;                 ;;  (ToolIdle ?tool)
;;                  (not (ToolAtJoint ?tool ?element1 ?element2))
;;                  (NoToolAtJoint ?element1 ?element2)
;;                  ; ! robot conf
;;                 ;;  (not (RobotAtConf ?conf1))
;;                 ;;  (RobotAtConf ?conf2)
;;                  ; ! switch for move
;;                  (CanMove)
;;             )
;;   )

;;   ;; tool is attached to the robot
;;   ;; a tool (gripper, clamp) can be placed if the goal place is clear of collision
;;   (:action place_clamp_at_joint
;;     :parameters (?tool ?element1 ?element2 ?conf1 ?conf2 ?traj)
;;     :precondition (and
;;                     ; ! robot state precondition
;;                     ;; (RobotAtConf ?conf1)
;;                     (Attached ?tool)
;;                     (IsClamp ?tool)
;;                     (Joint ?element1 ?element2)
;;                     ;; (ToolIdle ?tool)
;;                     (NoToolAtJoint ?element1 ?element2)
;;                     ; ! assembly state precondition
;;                     ; ! sampled
;;                     (PlaceToolAction ?tool ?conf1 ?conf2 ?traj)
;;                     )
;;     :effect (and (not (Attached ?tool))
;;                  (RobotToolChangerEmpty)
;;                  ; ! tool status
;;                 ;;  (not (ToolIdle ?tool))
;;                  (ToolAtJoint ?tool ?element1 ?element2)
;;                  (not (NoToolAtJoint ?element1 ?element2))
;;                  ; ! robot conf
;;                 ;;  (not (RobotAtConf ?conf1))
;;                 ;;  (RobotAtConf ?conf2)
;;                  ; ! switch for move
;;                  (CanMove)
;;                  )
;;   )

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;   (:derived (Connected ?element)
;;    (or (Grounded ?element)
;;        (exists (?ei) (and (Joint ?ei ?element)
;;                           (Assembled ?ei)
;;                           (Connected ?ei)
;;                      )
;;        )
;;    )
;;   )

;;   (:derived (AllToolAtJoints ?element)
;;    (forall (?ei) (imply (Joint ?ei ?element)
;;                         (exists (?tool) (ToolAtJoint ?tool ?ei ?element))
;;                  )
;;    )
;;   )

)
