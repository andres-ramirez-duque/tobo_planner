(define (problem proc1)
  (:domain ivprocdomain)
  (:objects 
    intro bye ivdescription bowout strategyenforce - activity
    bruno look bam shakeit macarena armdance happy calmdown babyshark oldmacdonald fivemonkeys happyandyouknow saxophone guitar asitwas idontcare whatdoyoumean blindinglights - activity
    belly cookies chocolate twinklestars taichi story - activity
  )
  (:init
    
    
    (introducing intro)
    
    (diverting bruno)
    (diverting look)
    (diverting bam)
    (diverting shakeit)
    (diverting macarena)
    (diverting happy)
    (diverting calmdown)
    (diverting saxophone)
    (diverting guitar)
    (diverting asitwas)
    ;(diverting idontcare)
    (diverting whatdoyoumean)
    (diverting blindinglights)
    
    ;(calming belly)
    (calming cookies)
    (calming chocolate)
    (calming twinklestars)
    (calming taichi)
    (calming story)
    
    (cd4 bruno)
    (cd4 look)
    (cd4 bam)
    (cd4 shakeit)
    (cd3 macarena)
    (cd5 happy)
    (cd5 calmdown)
    (cd5 saxophone)
    (cd4 guitar)
    (cd5 asitwas)
    ;(cd4 idontcare)
    (cd4 whatdoyoumean)
    (cd4 blindinglights)
    
    ;(cd1 belly) ; younger
    (cd2 cookies) 
    (cd1 chocolate)
    (cd1 taichi)
    (cd2 twinklestars)
    (cd2 story)
    
    
    (informing ivdescription)
    (informing strategyenforce)
    (withdrawl bowout)
    (procedureinfo ivdescription)
    (patientstrategyinfo strategyenforce)
    (finalise bye)
  )
  (:goal
    (and
      (introductioncomplete)
      (completedpreprocedure)
      (completedprocedure)
      (completedsitecheck)
      (debriefcomplete)
      (finishcomplete)
      (usedmaxdactivity)
    )
  )
)