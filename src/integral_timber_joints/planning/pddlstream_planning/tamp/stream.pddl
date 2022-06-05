(define (stream itj_clamp_only)

  (:stream sample-place_clamp_to_structure
    :inputs (?tool ?element1 ?element2)
    :domain (and
            (Clamp ?tool)
            (Joint ?element1 ?element2)
            (JointToolTypeMatch ?element1 ?element2 ?tool)
            (ClampedElement ?element2)
            )
    :outputs (?action)
    :certified (and
                (PlaceClampToStructureAction ?tool ?element1 ?element2 ?action)
               )
  )

  (:stream sample-pick_clamp_from_structure
    :inputs (?tool ?element1 ?element2)
    :domain (and
            (Clamp ?tool)
            (Joint ?element1 ?element2)
            (JointToolTypeMatch ?element1 ?element2 ?tool)
            (ClampedElement ?element2)
            )
    :outputs (?action)
    :certified (and
                (PickClampFromStructureAction ?tool ?element1 ?element2 ?action)
               )
  )


)
