(define (domain engivprocdomain)
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
    (informing ?a - activity)
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
    (requiressitecheck )
    (patientstrategyinfo ?a - activity)
    (givenstrategyinfo )
    (completedprocedure )
    (procedurehasfinished )
    (duringprocedure )
    (thirdperiod )
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
    (engaged )
    (forceaction )
    (requiresdisengage )
    (previouscalm )
    (previousdivert )
    (previousinform )
  )

  (:action readytostart
    :precondition (and
    (not (duringpreprocedure))
    (not (duringprocedure))
    (not (isreadytostart)))
    :effect
   (and
    (isreadytostart)
    (haswaited))
  )
  (:action ivintroduction
    :parameters (?a - activity)
    :precondition (and
    (isreadytostart)
    (not (duringpreprocedure))
    (not (duringprocedure))
    (introducing ?a)
    (not (engaged))
    (not (forceaction)))
    :effect
   (and
    (introductioncomplete)
    (doneactivity ?a)
    (engaged))
  )
  (:action reengage
    :precondition (and
    (not (engaged))
    (not (forceaction)))
    :effect
   (and
    (engaged))
  )
  (:action disengage
    :precondition (and
    (requiresdisengage)
    (engaged))
    :effect
   (and
    (not (engaged))
    (not (requiresdisengage))
    (not (forceaction)))
  )
  (:action introductionpause
    :precondition (and
    (not (duringpreprocedure))
    (not (duringprocedure))
    (introductioncomplete)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (intropausecomplete)
    (haspaused))
  )
  (:action ivstartpreprocedure
    :precondition (and
    (introductioncomplete)
    (intropausecomplete)
    (not (duringprocedure))
    (not (duringpreprocedure))
    (not (completedpreprocedure))
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (canmakedivertionplan)
    (duringpreprocedure)
    (tomanageanxiety))
  )
  (:action pameducateonprocedure
    :parameters (?a - activity)
    :precondition (and
    (not (previousinform))
    (not (amperforminganxietymanagement))
    (procedureinfo ?a)
    (informing ?a)
    (duringpreprocedure)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (previousinform)
    (not (previouscalm))
    (not (previousdivert))
    (haseducated)
    (givenprocedureinfo)
    (doneactivity ?a))
  )
  (:action ivcompletepreprocedure
    :precondition (and
    (hasmadedivertionplan)
    (amanxietymanagementcomplete)
    (hasdiverted)
    (haseducated)
    (duringpreprocedure)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (not (duringpreprocedure))
    (completedpreprocedure)
    (not (haseducated))
    (not (hasdiverted))
    (not (hascalmed)))
  )
  (:action ppstartanxietymanagement
    :precondition (and
    (haseducated)
    (duringpreprocedure)
    (tomanageanxiety)
    (not (amperforminganxietymanagement))
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (not (tomanageanxiety))
    (amrequiresanxietytest)
    (amperforminganxietymanagement))
  )
  (:action ppamtestanxiety
    :precondition (and
    (duringpreprocedure)
    (amrequiresanxietytest)
    (amperforminganxietymanagement)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (oneof (and
    (amanxietymanaged)
    (amanxietymanagementcomplete)
    (not (amperforminganxietymanagement))
    (okanxiety)) (and
    (amrequiresdivertion)
    (not (okanxiety))))
    (not (amrequiresanxietytest))
    (doneanxietytest))
  )
  (:action ppdivertor
    :parameters (?a - activity)
    :precondition (and
    (not (previousdivert))
    (not (amperforminganxietymanagement))
    (duringpreprocedure)
    (not (doneactivity ?a))
    (diverting ?a)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (previousdivert)
    (not (previouscalm))
    (not (previousinform))
    (hasdiverted)
    (doneactivity ?a))
  )
  (:action ppcalmer
    :parameters (?a - activity)
    :precondition (and
    (not (previouscalm))
    (not (amperforminganxietymanagement))
    (duringpreprocedure)
    (not (doneactivity ?a))
    (calming ?a)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (previouscalm)
    (not (previouscalm))
    (not (previousinform))
    (hascalmed)
    (doneactivity ?a))
  )
  (:action ppeamdivert
    :parameters (?a - activity)
    :precondition (and
    (duringpreprocedure)
    (amrequiresdivertion)
    (amperforminganxietymanagement)
    (not (doneactivity ?a))
    (diverting ?a)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (not (previousdivert))
    (not (previouscalm))
    (not (previousinform))
    (eamrequiresengagementtest)
    (not (amrequiresdivertion))
    (amrequirescalming)
    (hasdiverted)
    (doneactivity ?a))
  )
  (:action ppeamcalm
    :parameters (?a - activity)
    :precondition (and
    (duringpreprocedure)
    (okengagement)
    (eamdoneengagementtest)
    (amrequirescalming)
    (amperforminganxietymanagement)
    (calming ?a)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (previouscalm)
    (not (amrequirescalming))
    (amrequiresanxietyretest)
    (hascalmed)
    (doneactivity ?a))
  )
  (:action ppamretestanxiety
    :precondition (and
    (duringpreprocedure)
    (amrequiresanxietyretest)
    (amperforminganxietymanagement)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (oneof (and
    (amanxietymanaged)
    (amanxietymanagementcomplete)
    (okanxiety)) (and
    (amabort)
    (not (okanxiety))))
    (not (amrequiresanxietyretest))
    (not (amperforminganxietymanagement))
    (doneanxietytest))
  )
  (:action ivquerysitecheck
    :precondition (and
    (not (duringpreprocedure))
    (not (duringprocedure))
    (completedpreprocedure)
    (not (isreadyforprocedure))
    (not (duringsitecheck))
    (not (completedsitecheck))
    (not (requiressitecheck))
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (oneof (and
    (requiressitecheck)) (and
    (completedsitecheck)))
    (haswaited))
  )
  (:action ivstartsitecheck
    :precondition (and
    (introductioncomplete)
    (intropausecomplete)
    (completedpreprocedure)
    (not (duringprocedure))
    (not (duringpreprocedure))
    (requiressitecheck)
    (not (duringsitecheck))
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (duringsitecheck))
  )
  (:action completesitecheck
    :precondition (and
    (hascalmed)
    (givenstrategyinfo)
    (duringsitecheck)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (not (duringsitecheck))
    (completedsitecheck)
    (not (hascalmed))
    (not (haseducated)))
  )
  (:action relaxduringcheck
    :parameters (?a - activity)
    :precondition (and
    (duringsitecheck)
    (calming ?a)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (hascalmed)
    (doneactivity ?a))
  )
  (:action sceducateonprocedure
    :parameters (?a - activity)
    :precondition (and
    (patientstrategyinfo ?a)
    (duringsitecheck)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (haseducated)
    (givenstrategyinfo)
    (doneactivity ?a))
  )
  (:action readyforprocedure
    :precondition (and
    (not (duringpreprocedure))
    (not (duringprocedure))
    (not (duringsitecheck))
    (completedpreprocedure)
    (completedsitecheck)
    (not (isreadyforprocedure))
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (isreadyforprocedure)
    (haswaited))
  )
  (:action ivstartprocedure
    :precondition (and
    (isreadyforprocedure)
    (completedpreprocedure)
    (not (duringpreprocedure))
    (not (duringprocedure))
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (duringprocedure)
    (firstperiod))
  )
  (:action firstcompleteprocedure
    :precondition (and
    (hassatisfieddivertionplan)
    (duringprocedure)
    (firstperiod)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (oneof (and
    (procedurehasfinished)) (and
    (procedurestillrunning)
    (secondperiod)
    (not (hassatisfieddivertionplan))))
    (not (firstperiod)))
  )
  (:action secondcompleteprocedure
    :precondition (and
    (hassatisfieddivertionplan)
    (duringprocedure)
    (procedurestillrunning)
    (secondperiod)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (oneof (and
    (procedurehasfinished)
    (not (procedurestillrunning))) (and
    (procedurestillrunning)
    (thirdperiod)))
    (not (secondperiod)))
  )
  (:action waitforproceduretoend
    :precondition (and
    (procedurestillrunning)
    (thirdperiod)
    (not (waitedforproceduretoend))
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (oneof (and
    (procedurehasfinished)
    (not (procedurestillrunning))) (and
    (procedurecomplicationsoccurred)))
    (waitedforproceduretoend)
    (haswaited)
    (not (thirdperiod)))
  )
  (:action completeprocedure
    :precondition (and
    (procedurehasfinished)
    (not (procedurestillrunning))
    (duringprocedure)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (not (duringprocedure))
    (completedprocedure))
  )
  (:action pimplementdivertionplancalm
    :parameters (?a - activity)
    :precondition (and
    (duringprocedure)
    (not (doneactivity ?a))
    (hasmadedivertionplan)
    (uselectedcalming)
    (calming ?a)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (hassatisfieddivertionplan)
    (hascalmed)
    (doneactivity ?a))
  )
  (:action pimplementdivertionplandivert
    :parameters (?a - activity)
    :precondition (and
    (duringprocedure)
    (not (doneactivity ?a))
    (hasmadedivertionplan)
    (uselecteddivert)
    (diverting ?a)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (hassatisfieddivertionplan)
    (hasdiverted)
    (doneactivity ?a))
  )
  (:action ppmakedivertionplan
    :precondition (and
    (duringpreprocedure)
    (canmakedivertionplan)
    (not (hasmadedivertionplan))
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (oneof (and
    (uselecteddivert)) (and
    (uselectedcalming)))
    (hasmadedivertionplan))
  )
  (:action pmakedivertionplan
    :precondition (and
    (duringprocedure)
    (canmakedivertionplan)
    (not (hasmadedivertionplan))
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (oneof (and
    (uselecteddivert)) (and
    (uselectedcalming)))
    (hasmadedivertionplan))
  )
  (:action procedurepause
    :precondition (and
    (not (duringpreprocedure))
    (not (duringprocedure))
    (completedprocedure)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (procedurepausecomplete)
    (haspaused))
  )
  (:action ivdebrief
    :parameters (?a - activity)
    :precondition (and
    (procedurepausecomplete)
    (completedprocedure)
    (not (duringpreprocedure))
    (not (duringprocedure))
    (reward ?a))
    :effect
   (and
    (debriefcomplete)
    (doneactivity ?a))
  )
  (:action ivfinish
    :parameters (?a - activity)
    :precondition (and
    (debriefcomplete)
    (completedprocedure)
    (not (duringpreprocedure))
    (not (duringprocedure))
    (finalise ?a))
    :effect
   (and
    (finishcomplete)
    (doneactivity ?a))
  )
)
; 0.010478 0.000748 92.8612330597
