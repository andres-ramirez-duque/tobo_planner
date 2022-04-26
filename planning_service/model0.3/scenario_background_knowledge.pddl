(define (problem proc1)
  (:domain ivplacementsensing)
  (:objects 

    intro bye ivdescription song1 song2 dance1 dance2 leadmeditation taichi quiz idle1 idle2 idle3 - activity
    introstep preprocedure procedure debrief end - procstep
    distraction cognitivebehaviour proceduredescription intronau idle reward byenau - activitytype
  )
  (:init
    
    
    (stepproc introstep preprocedure)
    (stepproc preprocedure procedure)
    (stepproc procedure debrief)
    (stepproc debrief end)

    (anxietymitigating cognitivebehaviour)
    
    (activitycategory ivdescription proceduredescription)
    

    (activitycategory song1 distraction)
    (activitycategory song2 distraction)
    (activitycategory dance1 distraction)
    (activitycategory dance2 distraction)
    (activitycategory song1 reward)
    (activitycategory song2 reward)
    (activitycategory dance1 reward)
    (activitycategory dance2 reward)
    (activitycategory quiz distraction)

    (activitycategory leadmeditation cognitivebehaviour)
    (activitycategory taichi cognitivebehaviour)
    (activitycategory intro intronau)
    (activitycategory bye byenau)

    (activitycategory idle1 idle)
    (activitycategory idle2 idle)
    (activitycategory idle3 idle)
    
    (requiredcategory introstep intronau)
    (requiredcategory preprocedure cognitivebehaviour)
    (requiredcategory procedure distraction)
    (requiredcategory procedure cognitivebehaviour)
    (requiredcategory debrief reward)
    (requiredcategory end byenau)
        
    (anxietytest preprocedure)

    
  )
  (:goal
    (and
      (procedurestep)

      (procstage end)

    )
  )
)
