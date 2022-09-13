(define (domain ivprocdomain)
  (:types
    activity - object
  )
  (:predicates
    (doneactivity ?a - activity)
    (doneidle )
    (uselecteddivert )
    (uselectedcalming )
    (doneanxietytest )
    (okanxiety )
    (doneengagementtest )
    (okengagement )
    (haspaused )
    (haswaited )
    (introducing ?a - activity)
    (introductioncomplete )
    (hasdiverted )
    (diverting ?a - activity)
    (hascalmed )
    (calming ?a - activity)
    (amrequiresanxietytest )
    (amrequiresdivertion )
    (amrequirescalming )
    (amrequiresanxietyretest )
    (amperforminganxietymanagement )
    (amanxietymanagementcomplete )
    (amanxietymanaged )
    (amrequiresengagementtest )
    (amconeengagementtest )
    (tomanageanxiety )
    (amabort )
    (eamdoneengagementtest )
    (eamrequiresengagementtest )
    (withdrawl ?a - activity)
    (eamdisengaged )
    (canmakedivertionplan )
    (hasmadedivertionplan )
    (hassatisfieddivertionplan )
    (completedpreprocedure )
    (haseducated )
    (procedureinfo ?a - activity)
    (duringpreprocedure )
    (givenprocedureinfo )
    (duringsitecheck )
    (completedsitecheck )
    (patientstrategyinfo ?a - activity)
    (givenstrategyinfo )
    (completedprocedure )
    (duringprocedure )
    (thirdperiod)
    (firstperiod )
    (secondperiod )
    (procedurestillrunning )
    (procedurecomplicationsoccurred )
    (waitedforproceduretoend )
    (reward ?a - activity)
    (debriefcomplete )
    (finalise ?a - activity)
    (finishcomplete )
    (intropausecomplete )
    (isreadytostart )
    (isreadyforprocedure )
    (procedurepausecomplete )
  )

  (:action readytostart
    :precondition (and
    (not (duringpreprocedure ))
    (not (duringprocedure ))
    (not (isreadytostart )))
    :effect
   (and
    (isreadytostart )
    (haswaited ))
  )
  (:action ivintroduction
    :parameters (?a - activity)
    :precondition (and
    (isreadytostart )
    (not (duringpreprocedure ))
    (not (duringprocedure ))
    (introducing ?a))
    :effect
   (and
    (introductioncomplete )
    (doneactivity ?a))
  )
  (:action introductionpause
    :precondition (and
    (not (duringpreprocedure ))
    (not (duringprocedure ))
    (introductioncomplete ))
    :effect
   (and
    (intropausecomplete )
    (haspaused ))
  )
  (:action ivstartpreprocedure
    :precondition (and
    (introductioncomplete )
    (intropausecomplete )
    (not (duringprocedure ))
    (not (duringpreprocedure ))
    (not (completedpreprocedure )))
    :effect
   (and
    (canmakedivertionplan )
    (duringpreprocedure )
    (tomanageanxiety ))
  )
  (:action pameducateonprocedure
    :parameters (?a - activity)
    :precondition (and
    (not (amperforminganxietymanagement ))
    (procedureinfo ?a)
    (duringpreprocedure ))
    :effect
   (and
    (haseducated )
    (givenprocedureinfo )
    (doneactivity ?a))
  )
  (:action ivcompletepreprocedure
    :precondition (and
    (hasmadedivertionplan )
    (amanxietymanagementcomplete )
    (hasdiverted )
    (haseducated )
    (duringpreprocedure ))
    :effect
   (and
    (not (duringpreprocedure ))
    (completedpreprocedure )
    (not (haseducated ))
    (not (hasdiverted ))
    (not (hascalmed )))
  )
  (:action ppstartanxietymanagement
    :precondition (and
    (haseducated )
    (duringpreprocedure )
    (tomanageanxiety )
    (not (amperforminganxietymanagement )))
    :effect
   (and
    (not (tomanageanxiety ))
    (amrequiresanxietytest )
    (amperforminganxietymanagement ))
  )
  (:action ppamtestanxiety
    :precondition (and
    (duringpreprocedure )
    (amrequiresanxietytest )
    (amperforminganxietymanagement ))
    :effect
   (and
    (oneof (and
    (amanxietymanaged )
    (amanxietymanagementcomplete )
    (not (amperforminganxietymanagement ))
    (okanxiety )) (and
    (amrequiresdivertion )
    (not (okanxiety ))))
    (not (amrequiresanxietytest ))
    (doneanxietytest ))
  )
  (:action ppdivertor
    :parameters (?a - activity)
    :precondition (and
    (not (amperforminganxietymanagement ))
    (duringpreprocedure )
    (not (doneactivity ?a))
    (diverting ?a))
    :effect
   (and
    (hasdiverted )
    (doneactivity ?a))
  )
  (:action ppeamdivert
    :parameters (?a - activity)
    :precondition (and
    (duringpreprocedure )
    (amrequiresdivertion )
    (amperforminganxietymanagement )
    (not (doneactivity ?a))
    (diverting ?a))
    :effect
   (and
    (eamrequiresengagementtest )
    (not (amrequiresdivertion ))
    (amrequirescalming )
    (hasdiverted )
    (doneactivity ?a))
  )
  (:action ppeamcalm
    :parameters (?a - activity)
    :precondition (and
    (duringpreprocedure )
    (okengagement )
    (eamdoneengagementtest )
    (amrequirescalming )
    (amperforminganxietymanagement )
    (calming ?a))
    :effect
   (and
    (not (amrequirescalming ))
    (amrequiresanxietyretest )
    (hascalmed )
    (doneactivity ?a))
  )
  (:action ppamretestanxiety
    :precondition (and
    (duringpreprocedure )
    (amrequiresanxietyretest )
    (amperforminganxietymanagement ))
    :effect
   (and
    (oneof (and
    (amanxietymanaged )
    (amanxietymanagementcomplete )
    (okanxiety )) (and
    (amabort )
    (not (okanxiety ))))
    (not (amrequiresanxietyretest ))
    (not (amperforminganxietymanagement ))
    (doneanxietytest ))
  )
  (:action ppamengagementtest
    :precondition (and
    (duringpreprocedure )
    (eamrequiresengagementtest )
    (not (eamdoneengagementtest )))
    :effect
   (and
    (oneof (and
    (okengagement )) (and
    (eamdisengaged )
    (not (okengagement ))))
    (eamdoneengagementtest )
    (not (eamrequiresengagementtest ))
    (doneengagementtest ))
  )
  (:action ivlostengagement
    :parameters (?a - activity)
    :precondition (and
    (duringpreprocedure )
    (eamdisengaged )
    (withdrawl ?a))
    :effect
   (and
    (not (duringpreprocedure ))
    (completedpreprocedure )
    (completedsitecheck )
    (completedprocedure )
    (debriefcomplete )
    (finishcomplete )
    (amanxietymanagementcomplete )
    (doneactivity ?a))
  )
  (:action ivfailedtoimpact
    :parameters (?a - activity)
    :precondition (and
    (duringpreprocedure )
    (amabort )
    (withdrawl ?a))
    :effect
   (and
    (not (duringpreprocedure ))
    (completedpreprocedure )
    (completedsitecheck )
    (completedprocedure )
    (debriefcomplete )
    (finishcomplete )
    (amanxietymanagementcomplete )
    (doneactivity ?a))
  )
  (:action ivprocedurecomplications
    :precondition (and
    (procedurecomplicationsoccurred ))
    :effect
   (and
    (debriefcomplete )
    (finishcomplete )
    (not (procedurestillrunning ))
    (not (duringprocedure ))
    (completedprocedure )
    (doneidle ))
  )
  (:action ivquerysitecheck
    :precondition (and
    (not (duringpreprocedure ))
    (not (duringprocedure ))
    (completedpreprocedure )
    (not (isreadyforprocedure ))
    (not (duringsitecheck ))
    (not (completedsitecheck )))
    :effect
   (and
    (oneof (and
    (duringsitecheck )) (and
    (completedsitecheck )))
    (haswaited ))
  )
  (:action completesitecheck
    :precondition (and
    (hascalmed )
    (givenstrategyinfo )
    (duringsitecheck ))
    :effect
   (and
    (not (duringsitecheck ))
    (completedsitecheck )
    (not (hascalmed ))
    (not (haseducated )))
  )
  (:action relaxduringcheck
    :parameters (?a - activity)
    :precondition (and
    (duringsitecheck )
    (calming ?a))
    :effect
   (and
    (hascalmed )
    (doneactivity ?a))
  )
  (:action sceducateonprocedure
    :parameters (?a - activity)
    :precondition (and
    (patientstrategyinfo ?a)
    (duringsitecheck ))
    :effect
   (and
    (haseducated )
    (givenstrategyinfo )
    (doneactivity ?a))
  )
  (:action readyforprocedure
    :precondition (and
    (not (duringpreprocedure ))
    (not (duringprocedure ))
    (completedpreprocedure )
    (completedsitecheck )
    (not (isreadyforprocedure )))
    :effect
   (and
    (isreadyforprocedure )
    (haswaited ))
  )
  (:action ivstartprocedure
    :precondition (and
    (isreadyforprocedure )
    (completedpreprocedure )
    (not (duringpreprocedure ))
    (not (duringprocedure )))
    :effect
   (and
    (duringprocedure )
    (firstperiod ))
  )
  (:action firstcompleteprocedure
    :precondition (and
    (hassatisfieddivertionplan )
    (duringprocedure )
    (firstperiod ))
    :effect
   (and
    (oneof (and
    (completedprocedure )
    (not (duringprocedure ))
    ) (and
    (procedurestillrunning )
    (secondperiod )
    (not (hassatisfieddivertionplan ))))
    (not (firstperiod )))
  )
  (:action secondcompleteprocedure
    :precondition (and
    (hassatisfieddivertionplan )
    (duringprocedure )
    (secondperiod ))
    :effect
   (and
    (oneof (and
    (completedprocedure )
    (not (duringprocedure ))) (and
    (procedurestillrunning )))
    (not (secondperiod )) (thirdperiod))
  )
  (:action waitforproceduretoend
    :precondition (and
    (procedurestillrunning )
    (thirdperiod)
    (not (waitedforproceduretoend )))
    :effect
   (and
    (oneof (and
    (not (procedurestillrunning ))
    (not (thirdperiod))
    (not (duringprocedure ))
    (completedprocedure )) (and
    (procedurecomplicationsoccurred )))
    (waitedforproceduretoend )
    (haswaited ))
  )
  (:action pimplementdivertionplancalm
    :parameters (?a - activity)
    :precondition (and
    (duringprocedure )
    (not (doneactivity ?a))
    (hasmadedivertionplan )
    (uselectedcalming )
    (calming ?a))
    :effect
   (and
    (hassatisfieddivertionplan )
    (hascalmed )
    (doneactivity ?a))
  )
  (:action pimplementdivertionplandivert
    :parameters (?a - activity)
    :precondition (and
    (duringprocedure )
    (not (doneactivity ?a))
    (hasmadedivertionplan )
    (uselecteddivert )
    (diverting ?a))
    :effect
   (and
    (hassatisfieddivertionplan )
    (hasdiverted )
    (doneactivity ?a))
  )
  (:action ppmakedivertionplan
    :precondition (and
    (duringpreprocedure )
    (canmakedivertionplan )
    (not (hasmadedivertionplan )))
    :effect
   (and
    (oneof (and
    (uselecteddivert )) (and
    (uselectedcalming )))
    (hasmadedivertionplan ))
  )
  (:action pmakedivertionplan
    :precondition (and
    (duringprocedure )
    (canmakedivertionplan )
    (not (hasmadedivertionplan )))
    :effect
   (and
    (oneof (and
    (uselecteddivert )) (and
    (uselectedcalming )))
    (hasmadedivertionplan ))
  )
  (:action procedurepause
    :precondition (and
    (not (duringpreprocedure ))
    (not (duringprocedure ))
    (completedprocedure ))
    :effect
   (and
    (procedurepausecomplete )
    (haspaused ))
  )
  (:action ivdebrief
    :parameters (?a - activity)
    :precondition (and
    (procedurepausecomplete )
    (completedprocedure )
    (not (duringpreprocedure ))
    (not (duringprocedure ))
    (reward ?a))
    :effect
   (and
    (debriefcomplete )
    (doneactivity ?a))
  )
  (:action ivfinish
    :parameters (?a - activity)
    :precondition (and
    (debriefcomplete )
    (completedprocedure )
    (not (duringpreprocedure ))
    (not (duringprocedure ))
    (finalise ?a))
    :effect
   (and
    (finishcomplete )
    (doneactivity ?a))
  )
)
