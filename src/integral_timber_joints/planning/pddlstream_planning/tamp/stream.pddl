(define (stream itj_clamp_only)

  (:stream sample-place-element
    :inputs (?element)
    :domain (Element ?element)
    :outputs (?traj)
    :certified (and
                    (PlaceElementAction ?element ?traj)
                    (Traj ?traj)
                )
  )
)
