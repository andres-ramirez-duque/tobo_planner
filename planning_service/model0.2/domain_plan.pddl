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
    activity procstep activitytype
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
  )
  
  
  (:action doactivity1
    :parameters (?a - activity ?t - activitytype ?p - procstep)
    :precondition 
    (and
      (procstage ?p)
      (naustep)
      (not (done ?a))
      (activitycategory ?a ?t)
      (requiredcategory ?p ?t)
      
      (not (anxietytest ?p))
    )
    :effect
    (and
      (done ?a)
      (not (naustep))
      (procedurestep)
    )
  )
  (:action doactivity2
    :parameters (?a - activity ?t - activitytype ?p - procstep)
    :precondition 
    (and
      (procstage ?p)
      (naustep)
      (not (done ?a))
      (activitycategory ?a ?t)
      (requiredcategory ?p ?t)
      
      (okanxiety ?p)
    )
    :effect
    (and
      (done ?a)
      (not (naustep))
      (procedurestep)
    )
  )

    (:action mitigationactivity
    :parameters (?a - activity ?t - activitytype ?p - procstep)
    :precondition 
    (and
      (procstage ?p)
      (naustep)
      (not (done ?a))
      (activitycategory ?a ?t)
      (anxietymitigating ?t)
      (anxietytest ?p) 
      (not (okanxiety ?p))
    )
    :effect
    (and
      (done ?a)
      (not (naustep))
      (procedurestep)
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
      (not (procstage ?p1))
      (procstage ?p2)
      (not (procedurestep))
      (anxteststep)
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
      (not (procstage ?p1))
      (procstage ?p2)
      (not (procedurestep))
      (naustep)
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
      (not (anxteststep))
      (naustep)
      (oneof (okanxiety ?p) (and ))
    )
  )
)
