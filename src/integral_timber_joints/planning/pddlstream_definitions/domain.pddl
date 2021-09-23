(define (domain itj_clamp_only)
  (:requirements :strips :equality)
  (:predicates
    ; Static predicates (predicates that do not change over time)
    (Robot ?r)
    (Element ?e)
    (Joint ?e1 ?e2)
    ; (JointToolType ?e1 ?e2 ?tool)

    ; optional
    (Order ?e1 ?e2)

    (Gripper ?g)
    (Clamp ?c)
    (IsTool ?tool)

    (Pose ?pose)
    (Traj ?r ?t)

    (RobotConf ?r ?q)
    (ToolConf ?tool ?q)

    (PlaceAction ?r ?o ?p ?q1 ?q2)
    (PickAction ?r ?o ?p ?q1 ?q2)
    (MoveAction ?r ?q1 ?q2)

    ; Fluent predicates (predicates that change over time, which describes the state of the sytem)
    (Attached ?robot ?object) ; o can be element, gripper or clamp
    (ObjectAtPose ?object ?pose)
    (ToolAtPose ?tool ?pose)
    (RobotFlangeAtPose ?robot ?pose)

    (RobotAtConf ?robot ?conf)
    (ToolAtConf ?tool ?conf)

    (Assembled ?e)
    (AtRack ?o)

    (CanMove ?r)
  )

  (:action move
    :parameters (?r ?q1 ?q2 ?t2)
    :precondition (and
                        (Conf ?r ?q1)
                        (Conf ?r ?q2)
                        (Traj ?r ?t2)

                        (AtConf ?r ?q1)
                        (CanMove ?r)
                        (MoveAction ?r ?q1 ?q2 ?t2)
                        ;; collision constraint
                        (not (UnSafeTraj ?r ?t2))
                       )
    :effect (and
                 (not (AtConf ?r ?q1))
                 (AtConf ?r ?q2)
                 (not (CanMove ?r)) ; switch to avoid transit forever
                 )
  )

    place_element
    ; an element can only be placed if all the clamps are (attached) to the corresponding joints
    ; we can query a partial structure and a new element's connection joints using fluent
    pick_element

    place_tool(pose)
    ; tool is attached to the robot
    ; a tool (gripper, clamp) can be placed if the goal place is clear of collision
    pick_tool
    ; ToolAtPose()
    ; tool not attached
    ; HandEmpty

  (:action place
    :parameters (?r ?e ?q1 ?q2)
    :precondition (and
                    ;    (PlaceAction ?r ?e ?q1 ?q2 ?t)
                       (Grasped ?e)
                       ; (Stiff)
                       (Connected ?e)
                       ; e2 must be assembled before e
                       (forall (?e2) (imply (Order ?e2 ?e) (Assembled ?e2)))
                       ;; Collision constraint
                       ; (not (UnSafeTraj ?r ?t))
                       ;; comment the following two if no transit
                       (AtConf ?r ?q1) ; this will force a move action
                       (not (CanMove ?r))
                       )
    :effect (and (Assembled ?e)
                 (not (Grasped ?e))
                 (CanMove ?r)
                 )
  )

  (:action pick
    :parameters (?r ?e ?q1 ?q2)
    :precondition (and
                       (PlaceAction ?r ?e ?q1 ?q2 ?t)
                       (Assembled ?e)
                       ; (Stiff)
                       (Connected ?e)
                       ; e2 must be remove before e
                       (forall (?e2) (imply (Order ?e ?e2) (Removed ?e2)))
                       ;; Collision constraint
                       (not (UnSafeTraj ?r ?t))
                       ;; comment the following two if no transit
                    ;    (AtConf ?r ?q1) ; this will force a move action
                    ;    (not (CanMove ?r))
                       )
    :effect (and (Removed ?e)
                 (CanMove ?r)
                 (not (Assembled ?e))
                 )
  )

  (:derived (Connected ?e2)
   (or (Grounded ?e2)
       (exists (?e1) (and (Joined ?e1 ?e2)
                          (Assembled ?e1)
                          (Connected ?e1)
                     )
       )
   )
  )

)
