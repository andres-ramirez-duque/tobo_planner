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
    (highanxiety )
    (midanxiety )
    (lowanxiety )
    (noanxietyreading )
    (doneengagementtest )
    (okengagement )
    (haspaused )
    (haswaited )
    (bhighanxiety )
    (bmidanxiety )
    (blowanxiety )
    (haveanxietybelief )
    (requiresbeliefupdate )
    (hasdiverted )
    (diverting ?a - activity)
    (hascalmed )
    (calming ?a - activity)
    (amrequiresanxietytest )
    (amrequiresdivertion )
    (amrequirescalming )
    (amperforminganxietymanagement )
    (amanxietymanagementcomplete )
    (amrequiresengagementtest )
    (amconeengagementtest )
    (tomanageanxiety )
    (amabort )
    (requiresanxietymanagementcompletion )
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
    (requiressitecheck )
    (patientstrategyinfo ?a - activity)
    (givenstrategyinfo )
    (completedprocedure )
    (duringprocedure )
    (firstperiod )
    (secondperiod )
    (thirdperiod )
    (procedurestillrunning )
    (procedurehasfinished )
    (procedurecomplicationsoccurred )
    (waitedforproceduretoend )
    (introducing ?a - activity)
    (introductioncomplete )
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
    (not (completedpreprocedure ))
    (not (haveanxietybelief )))
    :effect
   (and
    (canmakedivertionplan )
    (duringpreprocedure ))
  )
  (:action pameducateonprocedurelowanxiety
    :parameters (?a - activity)
    :precondition (and
    (blowanxiety )
    (blowanxiety )
    (not (amperforminganxietymanagement ))
    (procedureinfo ?a)
    (duringpreprocedure )
    (haveanxietybelief ))
    :effect
   (and
    (haseducated )
    (givenprocedureinfo )
    (doneactivity ?a))
  )
  (:action pameducateonproceduremidanxiety
    :parameters (?a - activity)
    :precondition (and
    (bmidanxiety )
    (bmidanxiety )
    (not (amperforminganxietymanagement ))
    (procedureinfo ?a)
    (duringpreprocedure )
    (haveanxietybelief ))
    :effect
   (and
    (haseducated )
    (givenprocedureinfo )
    (doneactivity ?a))
  )
  (:action pameducateonprocedurehighanxiety
    :parameters (?a - activity)
    :precondition (and
    (bhighanxiety )
    (bhighanxiety )
    (not (amperforminganxietymanagement ))
    (procedureinfo ?a)
    (duringpreprocedure )
    (haveanxietybelief ))
    :effect
   (and
    (haseducated )
    (givenprocedureinfo )
    (doneactivity ?a))
  )
  (:action ivcompletepreprocedure
    :precondition (and
    (hasmadedivertionplan )
    (haveanxietybelief )
    (not (amperforminganxietymanagement ))
    (not (bhighanxiety ))
    (hasdiverted )
    (haseducated )
    (duringpreprocedure ))
    :effect
   (and
    (not (duringpreprocedure ))
    (completedpreprocedure )
    (not (haseducated ))
    (not (hasdiverted ))
    (not (hascalmed ))
    (not (haveanxietybelief )))
  )
  (:action ppstartanxietymanagement
    :precondition (and
    (duringpreprocedure )
    (not (amperforminganxietymanagement ))
    (doneanxietytest )
    (haveanxietybelief ))
    :effect
   (and
    (amperforminganxietymanagement )
    (amrequiresdivertion ))
  )
  (:action ppamtestanxiety
    :precondition (and
    (duringpreprocedure )
    (not (doneanxietytest )))
    :effect
   (and
    (oneof (oneof (lowanxiety ) (midanxiety )) (oneof (highanxiety ) (noanxietyreading )))
    (requiresbeliefupdate )
    (doneanxietytest ))
  )
  (:action ppdivertor
    :parameters (?a - activity)
    :precondition (and
    (not (amperforminganxietymanagement ))
    (duringpreprocedure )
    (not (requiresbeliefupdate ))
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
    (requiresanxietymanagementcompletion )
    (hascalmed )
    (doneactivity ?a))
  )
  (:action ppamanxietymanagementcompletionhigh2mid
    :precondition (and
    (duringpreprocedure )
    (bhighanxiety )
    (amperforminganxietymanagement )
    (requiresanxietymanagementcompletion ))
    :effect
   (and
    (not (bhighanxiety ))
    (bmidanxiety )
    (not (requiresanxietymanagementcompletion ))
    (not (amperforminganxietymanagement )))
  )
  (:action ppamanxietymanagementcompletionmid2low
    :precondition (and
    (duringpreprocedure )
    (bmidanxiety )
    (amperforminganxietymanagement )
    (requiresanxietymanagementcompletion ))
    :effect
   (and
    (not (bmidanxiety ))
    (blowanxiety )
    (not (requiresanxietymanagementcompletion ))
    (not (amperforminganxietymanagement )))
  )
  (:action ppamanxietymanagementcompletionlow2low
    :precondition (and
    (duringpreprocedure )
    (blowanxiety )
    (amperforminganxietymanagement )
    (requiresanxietymanagementcompletion ))
    :effect
   (and
    (not (requiresanxietymanagementcompletion ))
    (not (amperforminganxietymanagement )))
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
    (withdrawl ?a)
    (bhighanxiety ))
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
    (not (completedsitecheck ))
    (not (requiressitecheck )))
    :effect
   (and
    (oneof (and
    (requiressitecheck )) (and
    (completedsitecheck )))
    (haswaited ))
  )
  (:action ivstartsitecheck
    :precondition (and
    (not (duringpreprocedure ))
    (not (duringprocedure ))
    (completedpreprocedure )
    (not (isreadyforprocedure ))
    (not (duringsitecheck ))
    (requiressitecheck )
    (not (completedsitecheck )))
    :effect
   (and
    (duringsitecheck ))
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
    (not (duringsitecheck ))
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
    (procedurehasfinished )) (and
    (secondperiod )
    (not (hassatisfieddivertionplan ))
    (procedurestillrunning )))
    (not (firstperiod )))
  )
  (:action secondcompleteprocedure
    :precondition (and
    (hassatisfieddivertionplan )
    (duringprocedure )
    (procedurestillrunning )
    (secondperiod ))
    :effect
   (and
    (oneof (and
    (procedurehasfinished )
    (not (procedurestillrunning ))) (and
    (thirdperiod )))
    (not (secondperiod )))
  )
  (:action waitforproceduretoend
    :precondition (and
    (thirdperiod )
    (procedurestillrunning )
    (not (waitedforproceduretoend )))
    :effect
   (and
    (oneof (and
    (not (procedurestillrunning ))
    (procedurehasfinished )) (and
    (procedurecomplicationsoccurred )))
    (waitedforproceduretoend )
    (not (thirdperiod ))
    (haswaited ))
  )
  (:action completeprocedure
    :precondition (and
    (procedurehasfinished )
    (not (procedurestillrunning ))
    (duringprocedure ))
    :effect
   (and
    (not (duringprocedure ))
    (completedprocedure ))
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
    (not (amperforminganxietymanagement ))
    (not (requiresbeliefupdate ))
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
  (:action ppupdatebelief2lowanxiety
    :precondition (and
    (duringpreprocedure )
    (lowanxiety )
    (doneanxietytest )
    (requiresbeliefupdate ))
    :effect
   (and
    (not (lowanxiety ))
    (not (bmidanxiety ))
    (not (bhighanxiety ))
    (blowanxiety )
    (haveanxietybelief )
    (not (requiresbeliefupdate )))
  )
  (:action ppupdatebelief2midanxiety
    :precondition (and
    (duringpreprocedure )
    (midanxiety )
    (doneanxietytest )
    (requiresbeliefupdate ))
    :effect
   (and
    (not (midanxiety ))
    (not (blowanxiety ))
    (not (bhighanxiety ))
    (bmidanxiety )
    (haveanxietybelief )
    (not (requiresbeliefupdate )))
  )
  (:action ppupdatebelief2highanxiety
    :precondition (and
    (duringpreprocedure )
    (highanxiety )
    (doneanxietytest )
    (requiresbeliefupdate ))
    :effect
   (and
    (not (highanxiety ))
    (not (bmidanxiety ))
    (not (blowanxiety ))
    (bhighanxiety )
    (haveanxietybelief )
    (not (requiresbeliefupdate )))
  )
  (:action ppupdatebelief2previous
    :precondition (and
    (duringpreprocedure )
    (noanxietyreading )
    (doneanxietytest )
    (requiresbeliefupdate ))
    :effect
   (and
    (not (noanxietyreading ))
    (haveanxietybelief )
    (not (requiresbeliefupdate )))
  )
)
; 0.047803 0.003691 92.2787272765
