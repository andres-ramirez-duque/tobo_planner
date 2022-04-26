(define (problem proc1)
  (:domain ivplacementsensing)
  (:objects 

    intro bye ivdescription song1 song2 dance1 dance2 leadmeditation taichi quiz idle1 idle2 idle3 - activity
    introstep preprocedure procedure debrief end - procstep
    distraction cognitivebehaviour proceduredescription intronau idle reward byenau - activitytype
    low medium high - level
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
    (requiredcategory preprocedure distraction)
    (requiredcategory preprocedure proceduredescription)
    (requiredcategory procedure distraction)
    (requiredcategory procedure cognitivebehaviour)
    (requiredcategory debrief reward)
    (requiredcategory end byenau)
        
    (anxietytest preprocedure)
    
    (similar low low)
    (similar low medium)
    (similar medium medium)
    (similar medium low)
    (similar high high)
    (similar high medium)
    (similar medium high)
    
    ;; young
    (distractionstrength song1 high)
    (distractionstrength song2 medium)
    (distractionstrength dance1 high)
    (distractionstrength dance2 medium)
    (distractionstrength quiz medium)
    (distractionstrength leadmeditation medium)
    (distractionstrength taichi high)
    (distractionstrength intro medium)
    (distractionstrength bye low)
    (distractionstrength idle1 low)
    (distractionstrength idle2 low)
    (distractionstrength idle3 low)
    (distractionstrength ivdescription low)
    
    ;; old
    ;(distractionstrength song1 medium)
    ;(distractionstrength song2 high)
    ;(distractionstrength dance1 medium)
    ;(distractionstrength dance2 medium)
    ;(distractionstrength quiz high)
    ;(distractionstrength leadmeditation low)
    ;(distractionstrength taichi medium)
    ;(distractionstrength intro medium)
    ;(distractionstrength bye low)
    ;(distractionstrength idle1 low)
    ;(distractionstrength idle2 low)
    ;(distractionstrength idle3 low)
    ;(distractionstrength ivdescription high)
    
    (desiredstrength introstep low)
    (desiredstrength preprocedure high)
    (desiredstrength procedure high)
    (desiredstrength debrief medium)
    (desiredstrength end low)
    
  )
  (:goal
    (and
      (procedurestep)

      (procstage end)

    )
  )
)
