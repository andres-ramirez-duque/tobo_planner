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
    (diverting macarena)
    (diverting armdance)
    (diverting happy)
    (diverting calmdown)
    (diverting babyshark)
    (diverting fivemonkeys)
    (diverting happyandyouknow)
    (diverting saxophone)
    (diverting asitwas)
    (diverting idontcare)
    (diverting whatdoyoumean)
    ;(diverting blindinglights)    
    
    (calming belly)
    (calming cookies)
    ;(calming chocolate)
    (calming twinklestars)
    (calming taichi)
    (calming story)
    (calming oldmacdonald) ; maybe add a still in for the site check?
    
    (cd4 bruno)
    (cd4 look)
    (cd3 macarena)
    (cd3 armdance)
    (cd5 happy)
    (cd5 calmdown)
    (cd5 babyshark)
    (cd3 fivemonkeys)
    (cd3 happyandyouknow)
    (cd5 saxophone)
    (cd5 asitwas)
    (cd4 idontcare)
    (cd4 whatdoyoumean)
    ;(cd4 blindinglights)
    
    (cd1 belly) ; younger
    (cd2 cookies) ; younger,older
    (cd2 chocolate) ; older
    (cd1 taichi)
    (cd2 twinklestars)
    (cd2 story)
    (cd3 oldmacdonald)
    
    
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
