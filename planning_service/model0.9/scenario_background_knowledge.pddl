(define (problem proc1)
  (:domain ivprocdomain)
  (:objects 
    intro bye ivdescription song1 song2 dance1 dance2 leadmeditation taichi story quiz magic bowout strategyenforce - activity
  )
  (:init
    
    
    (introducing intro)
    (diverting dance1)
    (diverting dance2)
    (diverting song1)
    (diverting song2)
    (diverting quiz)
    (diverting magic)    
    (calming leadmeditation)
    (calming taichi)
    (calming story)
    (withdrawl bowout)
    (procedureinfo ivdescription)
    (patientstrategyinfo strategyenforce)
    (reward song1)
    (reward song2)
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

    )
  )
)
