(define (stream itj_clamp_only)

  (:stream sample-pick_beam_with_gripper
    ; beam_id, gripper_id
    :inputs (?element ?gripper)
    :domain (and (Element ?element) (Gripper ?gripper))
    :outputs (?action)
    :certified (and
                 (PickBeamWithGripperAction ?element ?gripper ?action)
               )
  )

)
