(define (stream itj_clamp_only)

  (:stream sample-place-element
    :inputs (?element)
    :domain (Element ?element)
    :fluents (Assembled)
    :outputs (?traj) 
    :certified (and
                    (PlaceElementAction ?element ?traj)
                    (Traj ?traj)
                )
  )

  ; TODO let the user implement this, even if it's deterministic
  ;; (:stream sample-joint-tool
  ;;   :inputs (?e1 ?e2)
  ;;   :domain (Joint ?e1 ?e2)
  ;;   ;; :fluents (Assembled)
  ;;   :outputs (?tool) 
  ;;   :certified (and
  ;;                   (Tool ?traj)
  ;;                   (JointToolTypeMatch ?element1 ?element2 ?tool)
  ;;               )
  ;; )
)
