;; Features:
;; Steps through procedure and has an activity at each step
;; Anxiety tests can be added to procedure steps
;; likes type of activity can be used after activity
;; Sensing for engagement at each procedure step
;; Sensing if procedure is finished
;;
;; Stategy is only currently partly implemented:
;; Idle for a turn if lost engagement
;; Don't perform the same action twice
;; Strategy for multi-step procedures with variable length
;; 
;;


(define (domain ivplacementsensing)
  (:requirements :non-deterministic :negative-preconditions :equality :typing)
  (:types
    activity procstep activitytype level
  )
  (:predicates
    (procedurestep)
    (naustep)
    (anxteststep)
    (procstage ?p - procstep)
    (stepproc ?p1 ?p2 - procstep)
    (done ?a - activity)
    (activitycategory ?a - activity ?t - activitytype)
    (requiredcategory ?a - procstep ?b - activitytype)
    (likes ?t - activitytype)
    (anxietymitigating ?t - activitytype)
    
    (anxietytest ?p - procstep)
    (okanxiety ?p - procstep)
    
    (similar ?l1 ?l2 - level)
    (distractionstrengthyoung ?a - activity ?l - level)
    (distractionstrengthold ?a - activity ?l - level)
    (desiredstrength ?p - procstep ?l - level)
    
    (isyoungerchild)
    
    (queried ?a - activity ?p - procstep)
    (uselectedaction ?p - procstep)
    (todoidle ?p - procstep)
    (todo ?a - activity ?p - procstep)
    (same ?a1 ?a2 - activity)
  )
    (:functions (total-cost) - number)
  
 ; (:action mitigation
 ;   :parameters (?a - activity ?p - procstep)
 ;   :precondition 
 ;   (and
 ;     (procstage ?p)
 ;     (naustep)
 ;     ;(not (done ?a))
 ;     (activitycategory ?a ?t)
 ;     (anxietymitigating ?t)
 ;     (anxietytest ?p) 
 ;     (not (okanxiety ?p))
 ;   )
 ;   :effect
 ;   (and
 ;     ;(done ?a)
 ;     (not (naustep))
 ;     (procedurestep)
 ;     (increase (total-cost) 1)
 ;   )
 ; )
  (:action mitigation
    :parameters (?p - procstep)
    :precondition 
    (and
      (procstage ?p)
      (naustep)
      (anxietytest ?p) 
      (not (okanxiety ?p))
    )
    :effect
    (and
      (not (naustep))
      (procedurestep)
      (increase (total-cost) 1)
    )
  )
  
  (:action doactivity1a
    :parameters (?a - activity ?t - activitytype ?p - procstep ?x - level)
    :precondition 
    (and
      (procstage ?p)
      (naustep)
      (not (done ?a))
      (activitycategory ?a ?t)
      (requiredcategory ?p ?t)
      
      (not (anxietytest ?p))
      
      (distractionstrengthyoung ?a ?x)
      (desiredstrength ?p ?x)
      
      (isyoungerchild)
      
      (uselectedaction ?p)
      (todo ?a ?p)
    )
    :effect
    (and
      (done ?a)
      (not (naustep))
      (procedurestep)
      (not (uselectedaction ?p))
      (not (todo ?a ?p))
      (increase (total-cost) 1)
    )
  )
  (:action doactivity2a
    :parameters (?a - activity ?t - activitytype ?p - procstep ?x - level)
    :precondition 
    (and
      (procstage ?p)
      (naustep)
      (not (done ?a))
      (activitycategory ?a ?t)
      (requiredcategory ?p ?t)
      
      (okanxiety ?p)
      
      (distractionstrengthyoung ?a ?x)
      (desiredstrength ?p ?x)
      
      (isyoungerchild)
      
      (uselectedaction ?p)
      (todo ?a ?p)
    )
    :effect
    (and
      (done ?a)
      (not (naustep))
      (procedurestep)
      (not (uselectedaction ?p))
      (not (todo ?a ?p))
      (increase (total-cost) 1)
    )
  )
    (:action doactivity1b
    :parameters (?a - activity ?t - activitytype ?p - procstep ?x ?y - level)
    :precondition 
    (and
      (procstage ?p)
      (naustep)
      (not (done ?a))
      (activitycategory ?a ?t)
      (requiredcategory ?p ?t)
      
      (not (anxietytest ?p))
      
      (distractionstrengthyoung ?a ?x)
      (desiredstrength ?p ?y)
      (similar ?x ?y)
      
      (isyoungerchild)
      
      (uselectedaction ?p)
      (todo ?a ?p)
    )
    :effect
    (and
      (done ?a)
      (not (naustep))
      (procedurestep)
      (not (uselectedaction ?p))
      (not (todo ?a ?p))
      (increase (total-cost) 5)
    )
  )
  (:action doactivity2b
    :parameters (?a - activity ?t - activitytype ?p - procstep ?x ?y - level)
    :precondition 
    (and
      (procstage ?p)
      (naustep)
      (not (done ?a))
      (activitycategory ?a ?t)
      (requiredcategory ?p ?t)
      
      (okanxiety ?p)
      
      (distractionstrengthyoung ?a ?x)
      (desiredstrength ?p ?y)
      (similar ?x ?y)
      
      (isyoungerchild)
      
      (uselectedaction ?p)
      (todo ?a ?p)
    )
    :effect
    (and
      (done ?a)
      (not (naustep))
      (procedurestep)
      (not (uselectedaction ?p))
      (not (todo ?a ?p))
      (increase (total-cost) 5)
    )
  )
    (:action doactivity1c
    :parameters (?a - activity ?t - activitytype ?p - procstep ?x ?y - level)
    :precondition 
    (and
      (procstage ?p)
      (naustep)
      (not (done ?a))
      (activitycategory ?a ?t)
      (requiredcategory ?p ?t)
      
      (not (anxietytest ?p))
      
      (distractionstrengthyoung ?a ?x)
      (desiredstrength ?p ?y)
      
      (isyoungerchild)
      
      (uselectedaction ?p)
      (todo ?a ?p)
    )
    :effect
    (and
      (done ?a)
      (not (naustep))
      (procedurestep)
      (not (uselectedaction ?p))
      (not (todo ?a ?p))
      (increase (total-cost) 10)
    )
  )
  (:action doactivity2c
    :parameters (?a - activity ?t - activitytype ?p - procstep ?x ?y - level)
    :precondition 
    (and
      (procstage ?p)
      (naustep)
      (not (done ?a))
      (activitycategory ?a ?t)
      (requiredcategory ?p ?t)
      
      (okanxiety ?p)
      
      (distractionstrengthyoung ?a ?x)
      (desiredstrength ?p ?y)

      (isyoungerchild)

      (uselectedaction ?p)
      (todo ?a ?p)
    )
    :effect
    (and
      (done ?a)
      (not (naustep))
      (procedurestep)
      (not (uselectedaction ?p))
      (not (todo ?a ?p))
      (increase (total-cost) 10)
    )
  )
  
    (:action doactivity1aold
    :parameters (?a - activity ?t - activitytype ?p - procstep ?x - level)
    :precondition 
    (and
      (procstage ?p)
      (naustep)
      (not (done ?a))
      (activitycategory ?a ?t)
      (requiredcategory ?p ?t)
      
      (not (anxietytest ?p))
      
      (distractionstrengthold ?a ?x)
      (desiredstrength ?p ?x)
      
      (not (isyoungerchild))
      
      (uselectedaction ?p)
      (todo ?a ?p)
    )
    :effect
    (and
      (done ?a)
      (not (naustep))
      (procedurestep)
      (not (uselectedaction ?p))
      (not (todo ?a ?p))
      (increase (total-cost) 1)
    )
  )
  (:action doactivity2aold
    :parameters (?a - activity ?t - activitytype ?p - procstep ?x - level)
    :precondition 
    (and
      (procstage ?p)
      (naustep)
      (not (done ?a))
      (activitycategory ?a ?t)
      (requiredcategory ?p ?t)
      
      (okanxiety ?p)
      
      (distractionstrengthold ?a ?x)
      (desiredstrength ?p ?x)
      
      (not (isyoungerchild))
      
      (uselectedaction ?p)
      (todo ?a ?p)
    )
    :effect
    (and
      (done ?a)
      (not (naustep))
      (procedurestep)
      (not (uselectedaction ?p))
      (not (todo ?a ?p))
      (increase (total-cost) 1)
    )
  )
    (:action doactivity1bold
    :parameters (?a - activity ?t - activitytype ?p - procstep ?x ?y - level)
    :precondition 
    (and
      (procstage ?p)
      (naustep)
      (not (done ?a))
      (activitycategory ?a ?t)
      (requiredcategory ?p ?t)
      
      (not (anxietytest ?p))
      
      (distractionstrengthold ?a ?x)
      (desiredstrength ?p ?y)
      (similar ?x ?y)
      
      (not (isyoungerchild))
      
      (uselectedaction ?p)
      (todo ?a ?p)
    )
    :effect
    (and
      (done ?a)
      (not (naustep))
      (procedurestep)
      (not (uselectedaction ?p))
      (not (todo ?a ?p))
      (increase (total-cost) 5)
    )
  )
  (:action doactivity2bold
    :parameters (?a - activity ?t - activitytype ?p - procstep ?x ?y - level)
    :precondition 
    (and
      (procstage ?p)
      (naustep)
      (not (done ?a))
      (activitycategory ?a ?t)
      (requiredcategory ?p ?t)
      
      (okanxiety ?p)
      
      (distractionstrengthold ?a ?x)
      (desiredstrength ?p ?y)
      (similar ?x ?y)
      
      (not (isyoungerchild))
      
      (uselectedaction ?p)
      (todo ?a ?p)
    )
    :effect
    (and
      (done ?a)
      (not (naustep))
      (procedurestep)
      (not (uselectedaction ?p))
      (not (todo ?a ?p))
      (increase (total-cost) 5)
    )
  )
    (:action doactivity1cold
    :parameters (?a - activity ?t - activitytype ?p - procstep ?x ?y - level)
    :precondition 
    (and
      (procstage ?p)
      (naustep)
      (not (done ?a))
      (activitycategory ?a ?t)
      (requiredcategory ?p ?t)
      
      (not (anxietytest ?p))
      
      (distractionstrengthold ?a ?x)
      (desiredstrength ?p ?y)
      
      (not (isyoungerchild))
      
      (uselectedaction ?p)
      (todo ?a ?p)
    )
    :effect
    (and
      (done ?a)
      (not (naustep))
      (procedurestep)
      (not (uselectedaction ?p))
      (not (todo ?a ?p))
      (increase (total-cost) 10)
    )
  )
  (:action doactivity2old
    :parameters (?a - activity ?t - activitytype ?p - procstep ?x ?y - level)
    :precondition 
    (and
      (procstage ?p)
      (naustep)
      (not (done ?a))
      (activitycategory ?a ?t)
      (requiredcategory ?p ?t)
      
      (okanxiety ?p)
      
      (distractionstrengthold ?a ?x)
      (desiredstrength ?p ?y)
      
      (not (isyoungerchild))
      
      (uselectedaction ?p)
      (todo ?a ?p)
    )
    :effect
    (and
      (done ?a)
      (not (naustep))
      (procedurestep)
      (not (uselectedaction ?p))
      (not (todo ?a ?p))
      (increase (total-cost) 10)
    )
  )
  
  
  
 (:action idledisengaged
    :parameters (?p - procstep)
    :precondition 
    (and
      (procstage ?p)
      (naustep)
      (uselectedaction ?p)
      (todoidle ?p)
    )
    :effect
    (and
      (not (uselectedaction ?p))
      (not (todoidle ?p))
      (not (naustep))
      (procedurestep)
      (increase (total-cost) 50)
    )
  )
  (:action idle1
    :parameters (?p - procstep)
    :precondition 
    (and
      (procstage ?p)
      (naustep)
      (not (anxietytest ?p))
      (not (uselectedaction ?p))
    )
    :effect
    (and
      (not (naustep))
      (procedurestep)
      (increase (total-cost) 1000)
    )
  )
  (:action idle2
    :parameters (?p - procstep)
    :precondition 
    (and
      (procstage ?p)
      (naustep)
      (okanxiety ?p)
      (not (uselectedaction ?p))
    )
    :effect
    (and
      (not (naustep))
      (procedurestep)
      (increase (total-cost) 1000)
    )
  )

  (:action progressprocstep1
    :parameters (?p1 ?p2 - procstep)
    :precondition 
    (and
      (procedurestep)
      (stepproc ?p1 ?p2)
      (procstage ?p1)
      (anxietytest ?p2)
    )
    :effect
    (and
      (not (procedurestep))
      (increase (total-cost) 1)
      (oneof
        (and 
          (not (procstage ?p1))
          (procstage ?p2)
          (anxteststep)
        )
        (naustep)
      )
    )
  )
  
  (:action progressprocstep2
    :parameters (?p1 ?p2 - procstep)
    :precondition 
    (and
      (procedurestep)
      (stepproc ?p1 ?p2)
      (procstage ?p1)
      (not (anxietytest ?p2))
    )
    :effect
    (and
      (naustep)
      (not (procedurestep))
      (increase (total-cost) 1)
      (oneof
        (and 
          (not (procstage ?p1))
          (procstage ?p2)
        )
        (and )
      )
    )
  )

  (:action bypassactivityprefq
  :parameters (?a1 - activity ?t1 - activitytype ?p - procstep)
    :precondition 
    (and
      (activitycategory ?a1 ?t1)
      (requiredcategory ?p ?t1)
      (not (queried ?a1 ?p))
      (not (uselectedaction ?p))
      
      (not (done ?a1))
    )
    :effect
    (and
      (uselectedaction ?p)
      (todo ?a1 ?p)
      (queried ?a1 ?p)
        
      (increase (total-cost) 50)
    )
  )

  (:action qactivitypreference
    :parameters (?a1 ?a2 - activity ?t1 ?t2 - activitytype ?p - procstep)
    :precondition 
    (and
      (not (queried ?a1 ?p))
      (not (queried ?a2 ?p))
      
      (not (done ?a1))
      (not (done ?a2))
      
      (not (same ?a1 ?a2))
      
      (activitycategory ?a1 ?t1)
      (requiredcategory ?p ?t1)
      (activitycategory ?a2 ?t2)
      (requiredcategory ?p ?t2)
      
      (not (uselectedaction ?p))
    )
    :effect
    (and
      (increase (total-cost) 1)
      (oneof
        (oneof
         (and ; user chooses ?a1
          (uselectedaction ?p)
          (todo ?a1 ?p)
          (queried ?a1 ?p)
          (queried ?a2 ?p)
         )
         (and ; user chooses ?a2
          (uselectedaction ?p)
          (todo ?a2 ?p)
          (queried ?a1 ?p)
          (queried ?a2 ?p)
         )
        )
       (oneof
        (and ; disengaged from question
          (uselectedaction ?p)
          (todoidle ?p)
          (queried ?a1 ?p)
          (queried ?a2 ?p)
        )
        (and ) ; didn't understand response
       )
      )
    )
  )

  (:action anxietytest
    :parameters (?p - procstep)
    :precondition 
    (and
      (anxteststep)
      (anxietytest ?p)
      (procstage ?p)
    )
    :effect
    (and
      (increase (total-cost) 1)
      (not (anxteststep))
      (naustep)
      (oneof (okanxiety ?p) (and ))
    )
  )
)
