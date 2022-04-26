(define (problem proc1)
  (:domain ivplacementsensing)
  (:objects 

    intro bye ivdescription song1 song2 dance1 dance2 leadmeditation taichi quiz - activity
    introstep preprocedure procedure debrief end - procstep
    distraction cognitivebehaviour proceduredescription intronau reward byenau - activitytype
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
    (distractionstrengthyoung song1 high)
    (distractionstrengthyoung song2 medium)
    (distractionstrengthyoung dance1 high)
    (distractionstrengthyoung dance2 medium)
    (distractionstrengthyoung quiz medium)
    (distractionstrengthyoung leadmeditation medium)
    (distractionstrengthyoung taichi high)
    (distractionstrengthyoung intro medium)
    (distractionstrengthyoung bye low)
    (distractionstrengthyoung ivdescription low)
    
    ;; old
    (distractionstrengthold song1 medium)
    (distractionstrengthold song2 high)
    (distractionstrengthold dance1 medium)
    (distractionstrengthold dance2 medium)
    (distractionstrengthold quiz high)
    (distractionstrengthold leadmeditation low)
    (distractionstrengthold taichi medium)
    (distractionstrengthold intro medium)
    (distractionstrengthold bye low)
    (distractionstrengthold ivdescription high)
    
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
