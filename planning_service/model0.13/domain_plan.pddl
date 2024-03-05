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
    (secondsitecheck)
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
    (maxdselected ?a - activity)
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
    (usedmaxdactivity)
    (uselected ?a - activity)
    (cd5 ?a - activity)
    (cd4 ?a - activity)
    (cd3 ?a - activity)
    (cd2 ?a - activity)
    (cd1 ?a - activity)
    (age)
    (iv)
  )
  (:functions (total-cost) - number)
  (:action readytostart
    :precondition (and
    (not (duringpreprocedure))
    (not (duringprocedure))
    (not (isreadytostart)))
    :effect
   (and
    (isreadytostart)
    (haswaited)
    (increase (total-cost) 1))
  )
  (:action ivintroduction
    :parameters (?a - activity)
    :precondition (and
    (isreadytostart)
    (not (duringpreprocedure))
    (not (duringprocedure))
    (not (introductioncomplete))
    (introducing ?a)
    (not (engaged))
    ;(not (forceaction))
    )
    :effect
   (and
    (introductioncomplete)
    (doneactivity ?a)
    (engaged)
    (increase (total-cost) 1))
  )
  (:action reengage
    :precondition (and
    (not (engaged))
    (not (forceaction))
    (not (amperforminganxietymanagement))
    (not (duringpreprocedure))
    (not (duringsitecheck))
    (completedpreprocedure ))
    :effect
   (and
    (engaged)
    (increase (total-cost) 1))
  )
  ;(:action ppreengage
  ;  :precondition (and
  ;  (not (engaged))
  ;  (not (forceaction))
  ;  (not (amperforminganxietymanagement))
  ;  (duringpreprocedure)
  ;  )
  ;  :effect
  ; (and
  ;  (engaged))
  ;)
  (:action ppreengage
    :parameters (?a1 ?a2 - activity)
    :precondition 
    (and
      (not (= ?a1 ?a2))
      (not (engaged))
      (not (doneactivity ?a1))
      (not (doneactivity ?a2))
      (cd5 ?a2)
      (diverting ?a2)
      (calming ?a1)
    (not (forceaction))
    (not (amperforminganxietymanagement))
    (duringpreprocedure)
    )
    :effect
    (and
      (oneof
        (uselected ?a1)
        (uselected ?a2)
      )
      (engaged)
      (forceaction)
      (increase (total-cost) 1)
    )
  )
  (:action ppreengagel
    :parameters (?a1 ?a2 - activity)
    :precondition 
    (and
      (not (= ?a1 ?a2))
      (not (engaged))
      (not (doneactivity ?a1))
      (not (doneactivity ?a2))
      (diverting ?a2)
      (calming ?a1)
    (not (forceaction))
    (not (amperforminganxietymanagement))
    (duringpreprocedure)
    )
    :effect
    (and
      (oneof
        (uselected ?a1)
        (uselected ?a2)
      )
      (engaged)
      (forceaction)
      (increase (total-cost) 10)
    )
  )
  (:action ppreengagenc
    :parameters (?a2 - activity)
    :precondition 
    (and
      (not (engaged))
      (not (doneactivity ?a2))
      (cd5 ?a2)
      (diverting ?a2)
    (not (forceaction))
    (not (amperforminganxietymanagement))
    (duringpreprocedure)
    )
    :effect
    (and
      (uselected ?a2)
      (engaged)
      (forceaction)
      (increase (total-cost) 20)
    )
  )
  (:action screengage
    :parameters (?a1 ?a2 - activity)
    :precondition 
    (and
      (not (= ?a1 ?a2))
      (not (engaged))
      (not (doneactivity ?a1))
      (not (doneactivity ?a2))
      (calming ?a2)
      (calming ?a1)
    (not (forceaction))
    (not (amperforminganxietymanagement))
    (not (duringpreprocedure))
    (duringsitecheck)
    )
    :effect
    (and
      (oneof
        (uselected ?a1)
        (uselected ?a2)
      )
      (engaged)
      (forceaction)
      (increase (total-cost) 1)
    )
  )
  (:action forceddivertor
    :parameters (?a - activity)
    :precondition (and
    (not (amperforminganxietymanagement))
    ;(duringpreprocedure)
    (not (doneactivity ?a))
    (diverting ?a)
    (engaged)
    (forceaction)
    (uselected ?a)
    )
    :effect
   (and
    (previousdivert)
    (not (previouscalm))
    (not (previousinform))
    (not (uselected ?a))
    (hasdiverted)
    (not (forceaction))
    (doneactivity ?a)
    (increase (total-cost) 1)
    )
  )
  (:action forcedcalmer
    :parameters (?a - activity)
    :precondition (and
    (not (amperforminganxietymanagement))
    ;(duringpreprocedure)
    (not (doneactivity ?a))
    (calming ?a)
    (engaged)
    (uselected ?a)
    (forceaction)
    )
    :effect
   (and
    (previouscalm)
    (not (previousdivert))
    (not (previousinform))
    (not (uselected ?a))
    (not (forceaction))
    (hascalmed)
    (doneactivity ?a)
    (increase (total-cost) 1)
    )
  )
  (:action forcedbowout
    :parameters (?a - activity)
    :precondition (and
    ;(duringpreprocedure)
    (not (doneactivity ?a))
    (withdrawl ?a)
    ;(engaged)
    (uselected ?a)
    (forceaction)
    )
    :effect
   (and
    (not (duringpreprocedure ))
    (completedpreprocedure )
    (completedsitecheck )
    (completedprocedure )
    (debriefcomplete )
    (finishcomplete )
    (amanxietymanagementcomplete )
    (not (amperforminganxietymanagement))
    (doneactivity ?a)
    (increase (total-cost) 1000)
    )
  )
  (:action amreengage
    :precondition (and
    (not (engaged))
    (not (forceaction))
    (amperforminganxietymanagement)
    (doneanxietytest)
    (okanxiety)
    )
    :effect
   (and
    (engaged)
    (increase (total-cost) 1))
  )
  (:action disengage
    :precondition (and
    (requiresdisengage)
    (engaged)
    (not (previousinform))
    (not (amperforminganxietymanagement)))
    :effect
   (and
    (not (engaged))
    (not (requiresdisengage))
    (not (forceaction))
    (increase (total-cost) 1))
  )
  (:action disengageniv
    :precondition (and
    (requiresdisengage)
    (engaged)
    (previousinform)
    (not (amperforminganxietymanagement)))
    :effect
   (and
    (not (iv))
    (not (engaged))
    (not (requiresdisengage))
    (not (forceaction))
    (increase (total-cost) 1))
  )
  (:action amdisengage
    :precondition (and
    (requiresdisengage)
    (engaged)
    (amperforminganxietymanagement))
    :effect
   (and
    (not (engaged))
    (not (requiresdisengage))
    (not (forceaction))
    (increase (total-cost) 1))
  )
  (:action ivlostengagement
    :parameters (?a - activity)
    :precondition (and
    (duringpreprocedure )
    (amperforminganxietymanagement)
    (doneanxietytest)
    (not (okanxiety))
    (not (engaged))
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
    (not (amperforminganxietymanagement))
    (doneactivity ?a)
    (increase (total-cost) 1000))
  )
  
  (:action introductionpause
    :precondition (and
    (not (duringpreprocedure))
    (not (duringprocedure))
    (introductioncomplete)
    (engaged)
    ;(not (forceaction))
    )
    :effect
   (and
    (intropausecomplete)
    (haspaused)
    (increase (total-cost) 1))
  )
  (:action ivstartpreprocedure
    :precondition (and
    (introductioncomplete)
    (intropausecomplete)
    (not (duringprocedure))
    (not (duringpreprocedure))
    (not (completedpreprocedure))
    (engaged)
    ;(not (forceaction))
    )
    :effect
   (and
    (canmakedivertionplan)
    (duringpreprocedure)
    (tomanageanxiety)
    (increase (total-cost) 1))
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
    (not (forceaction))
    (age)
    (iv))
    :effect
   (and
    (previousinform)
    (not (previouscalm))
    (not (previousdivert))
    (haseducated)
    (givenprocedureinfo)
    (doneactivity ?a)
    (increase (total-cost) 1))
  )
  (:action pameducateonprocedurey
    :parameters (?a - activity)
    :precondition (and
    (not (previousinform))
    (not (amperforminganxietymanagement))
    (procedureinfo ?a)
    (informing ?a)
    (duringpreprocedure)
    (engaged)
    (not (forceaction))
    (not (age))
    (iv))
    :effect
   (and
    (previousinform)
    (not (previouscalm))
    (not (previousdivert))
    (haseducated)
    (givenprocedureinfo)
    (doneactivity ?a)
    (increase (total-cost) 1))
  )
  (:action ivcompletepreprocedure
    :precondition (and
    (iv)
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
    (not (hascalmed))
    (increase (total-cost) 1))
  )
  (:action ivcompletepreprocedureniv
    :precondition (and
    (not (iv))
    (hasmadedivertionplan)
    (amanxietymanagementcomplete)
    (hasdiverted)
    (duringpreprocedure)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (not (duringpreprocedure))
    (completedpreprocedure)
    (not (haseducated))
    (not (hasdiverted))
    (not (hascalmed))
    (increase (total-cost) 1))
  )
  (:action ppstartanxietymanagement
    :precondition (and
    (haseducated)
    (duringpreprocedure)
    (tomanageanxiety)
    (not (amperforminganxietymanagement))
    (engaged)
    (not (forceaction))
    (iv))
    :effect
   (and
    (not (tomanageanxiety))
    (amrequiresanxietytest)
    (amperforminganxietymanagement)
    (increase (total-cost) 1))
  )
  (:action ppstartanxietymanagementniv
    :precondition (and
    (duringpreprocedure)
    (tomanageanxiety)
    (not (amperforminganxietymanagement))
    (engaged)
    (not (forceaction))
    (not (iv)))
    :effect
   (and
    (not (tomanageanxiety))
    (amrequiresanxietytest)
    (amperforminganxietymanagement)
    (increase (total-cost) 1))
  )
  (:action ppamtestanxiety
    :precondition (and
    (duringpreprocedure)
    (amrequiresanxietytest)
    (amperforminganxietymanagement)
    (engaged))
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
    (doneanxietytest)
    (increase (total-cost) 1))
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
    (doneactivity ?a)
    (increase (total-cost) 1))
  )
  (:action ppdivertormax
    :parameters (?a - activity)
    :precondition (and
    (not (previousdivert))
    (not (amperforminganxietymanagement))
    (duringpreprocedure)
    (not (doneactivity ?a))
    (maxdselected ?a)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (usedmaxdactivity)
    (previousdivert)
    (not (previouscalm))
    (not (previousinform))
    (hasdiverted)
    (doneactivity ?a)
    (increase (total-cost) 20))
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
    (not (previousdivert))
    (not (previousinform))
    (hascalmed)
    (doneactivity ?a)
    (increase (total-cost) 1))
  )
  (:action ppeamdivert
    :parameters (?a - activity)
    :precondition (and
    (duringpreprocedure)
    (amrequiresdivertion)
    (amperforminganxietymanagement)
    (not (doneactivity ?a))
    (diverting ?a)
    (cd5 ?a)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (not (previousdivert))
    (not (previouscalm))
    (not (previousinform))
    (not (amrequiresdivertion))
    (amrequirescalming)
    (hasdiverted)
    (doneactivity ?a)
    (increase (total-cost) 1))
  )
  (:action ppeamdivertmax
    :parameters (?a - activity)
    :precondition (and
    (duringpreprocedure)
    (amrequiresdivertion)
    (amperforminganxietymanagement)
    (not (doneactivity ?a))
    (maxdselected ?a)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (usedmaxdactivity)
    (not (previousdivert))
    (not (previouscalm))
    (not (previousinform))
    (not (amrequiresdivertion))
    (amrequirescalming)
    (hasdiverted)
    (doneactivity ?a)
    (increase (total-cost) 10))
  )
  
  (:action ppeamdivertl
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
    (not (amrequiresdivertion))
    (amrequirescalming)
    (hasdiverted)
    (doneactivity ?a)
    (increase (total-cost) 10))
  )
  (:action ppeamcalm
    :parameters (?a - activity)
    :precondition (and
    (duringpreprocedure)
    (amrequirescalming)
    (amperforminganxietymanagement)
    (calming ?a)
    (engaged)
    (not (doneactivity ?a))
    (not (forceaction))
    (cd1 ?a))
    :effect
   (and
    (previouscalm)
    (not (amrequirescalming))
    (amrequiresanxietyretest)
    (hascalmed)
    (doneactivity ?a)
    (increase (total-cost) 1))
  )
  (:action ppeamcalml
    :parameters (?a - activity)
    :precondition (and
    (duringpreprocedure)
    (amrequirescalming)
    (amperforminganxietymanagement)
    (calming ?a)
    (engaged)
    (not (doneactivity ?a))
    (not (forceaction)))
    :effect
   (and
    (previouscalm)
    (not (amrequirescalming))
    (amrequiresanxietyretest)
    (hascalmed)
    (doneactivity ?a)
    (increase (total-cost) 10))
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
    (doneanxietytest)
    (increase (total-cost) 1))
  )
  (:action ivfailedtoimpact
    :parameters (?a - activity)
    :precondition (and
    (duringpreprocedure)
    (amabort)
    (withdrawl ?a)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (not (duringpreprocedure))
    (completedpreprocedure)
    (completedsitecheck)
    (completedprocedure)
    (debriefcomplete)
    (finishcomplete)
    (amanxietymanagementcomplete)
    (doneactivity ?a)
    (increase (total-cost) 1000))
  )
  (:action ivprocedurecomplications
    :precondition (and
    (procedurecomplicationsoccurred))
    :effect
   (and
    (debriefcomplete)
    (finishcomplete)
    (not (procedurestillrunning))
    (not (duringprocedure))
    (completedprocedure)
    (doneidle)
    (increase (total-cost) 1000))
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
    ;(engaged)
    ;(not (forceaction))
    )
    :effect
   (and
    (oneof (and
    (requiressitecheck)) (and
    (completedsitecheck)))
    (haswaited)
    (increase (total-cost) 1))
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
    ;(engaged)
    ;(not (forceaction))
    )
    :effect
   (and
    (duringsitecheck)
    (not (requiressitecheck))
    (increase (total-cost) 1))
  )
  (:action completesitecheck
    :precondition (and
    (hascalmed)
    (givenstrategyinfo)
    (duringsitecheck)
    ;(engaged)
    (not (forceaction))
    (not (secondsitecheck)))
    :effect
   (and
    (oneof (and
    (not (duringsitecheck))
    (not (haseducated))
    (completedsitecheck)) 
    (and (secondsitecheck))
    )
    (not (hascalmed))
    (increase (total-cost) 1))
  )
  (:action completesitecheck2
    :precondition (and
    (hascalmed)
    (givenstrategyinfo)
    (duringsitecheck)
    ;(engaged)
    (secondsitecheck)
    (not (forceaction)))
    :effect
   (and
    (not (duringsitecheck))
    (not (haseducated))
    (completedsitecheck)
    (not (hascalmed))
    (increase (total-cost) 1))
  )
  (:action relaxduringcheck
    :parameters (?a - activity)
    :precondition (and
    (duringsitecheck)
    (calming ?a)
    (engaged)
    (not (doneactivity ?a))
    (not (forceaction)))
    :effect
   (and
    (hascalmed)
    (doneactivity ?a)
    (increase (total-cost) 1))
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
    (doneactivity ?a)
    (increase (total-cost) 1))
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
    (not (forceaction))
    )
    :effect
   (and
    (isreadyforprocedure)
    (haswaited)
    (increase (total-cost) 1))
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
    (firstperiod)
    (increase (total-cost) 1))
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
    (not (firstperiod))
    (increase (total-cost) 1))
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
    (not (secondperiod))
    (increase (total-cost) 1))
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
    (not (thirdperiod))
    (increase (total-cost) 1))
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
    (completedprocedure)
    (increase (total-cost) 1))
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
    (not (forceaction))
    (cd1 ?a))
    :effect
   (and
    (hassatisfieddivertionplan)
    (hascalmed)
    (doneactivity ?a)
    (increase (total-cost) 1))
  )
  (:action pimplementdivertionplancalml
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
    (doneactivity ?a)
    (increase (total-cost) 20))
  )
  (:action pimplementdivertionplandivert
    :parameters (?a - activity)
    :precondition (and
    (duringprocedure)
    (not (doneactivity ?a))
    (hasmadedivertionplan)
    (uselecteddivert)
    ;(diverting ?a)
    (engaged)
    (not (forceaction))
    (maxdselected ?a))
    :effect
   (and
    (usedmaxdactivity)
    (hassatisfieddivertionplan)
    (hasdiverted)
    (doneactivity ?a)
    (increase (total-cost) 1))
  )
  (:action pimplementdivertionplandivert5
    :parameters (?a - activity)
    :precondition (and
    (duringprocedure)
    (not (doneactivity ?a))
    (hasmadedivertionplan)
    (uselecteddivert)
    (diverting ?a)
    (engaged)
    (not (forceaction))
    (cd5 ?a))
    :effect
   (and
    (hassatisfieddivertionplan)
    (hasdiverted)
    (doneactivity ?a)
    (increase (total-cost) 10))
  )
  (:action pimplementdivertionplandivertl
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
    (doneactivity ?a)
    (increase (total-cost) 20))
  )
  (:action pbadlyimplementeddivertplan
    :parameters (?a - activity)
    :precondition (and
    (duringprocedure)
    (not (doneactivity ?a))
    (hasmadedivertionplan)
    (uselecteddivert)
    (calming ?a)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (hassatisfieddivertionplan)
    (hascalmed)
    (doneactivity ?a)
    (increase (total-cost) 100))
  )
  (:action pbadlyimplementedcalmplan
    :parameters (?a - activity)
    :precondition (and
    (duringprocedure)
    (not (doneactivity ?a))
    (hasmadedivertionplan)
    (uselectedcalming)
    (diverting ?a)
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (hassatisfieddivertionplan)
    (hasdiverted)
    (doneactivity ?a)
    (increase (total-cost) 100))
  )
  (:action ppmakedivertionplan
    :precondition (and
    (duringpreprocedure)
    (canmakedivertionplan)
    (not (hasmadedivertionplan))
    (not (amperforminganxietymanagement))
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (oneof (and
    (uselecteddivert)) (and
    (uselectedcalming)))
    (hasmadedivertionplan)
    (increase (total-cost) 1))
  )
  (:action pmakedivertionplan
    :precondition (and
    (duringprocedure)
    (canmakedivertionplan)
    (not (hasmadedivertionplan))
    (not (amperforminganxietymanagement))
    (engaged)
    (not (forceaction)))
    :effect
   (and
    (oneof (and
    (uselecteddivert)) (and
    (uselectedcalming)))
    (hasmadedivertionplan)
    (increase (total-cost) 1))
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
    (haspaused)
    (increase (total-cost) 1))
  )
  (:action ivdebrief
    :parameters (?a - activity)
    :precondition (and
    (procedurepausecomplete)
    (completedprocedure)
    (not (duringpreprocedure))
    (not (duringprocedure))
    (not (doneactivity ?a))
    (reward ?a))
    :effect
   (and
    (debriefcomplete)
    (doneactivity ?a)
    (increase (total-cost) 1))
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
    (doneactivity ?a)
    (increase (total-cost) 1))
  )
  (:action undodoneactivity
    :parameters (?a - activity)
    :precondition (doneactivity ?a)
    :effect (and (not (doneactivity ?a)) (increase (total-cost) 10000)))
)
; 0.004382 0.000397 90.9402099498
