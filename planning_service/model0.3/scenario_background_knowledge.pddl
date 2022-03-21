(define (problem proc1)
  (:domain ivplacementsensing)
  (:objects 
    intro bye song1 song2 dance1 dance2 leadmeditation taichi quiz idle1 idle2 idle3 - activity
    preentrance staffintro prep1 tourniquet prep2 iv tidy goodbye postleave end - procstep
    distraction cognitivebehaviour intronau idle reward byenau - activitytype
    low medium high - level
  )
  (:init
    (naustep)
    
    (stepproc preentrance staffintro)
    (stepproc staffintro prep1)
    (stepproc prep1 tourniquet)
    (stepproc tourniquet prep2)
    (stepproc prep2 iv)
    (stepproc iv tidy)
    (stepproc tidy goodbye)
    (stepproc goodbye postleave)
    (stepproc postleave end)

    (anxietymitigating cognitivebehaviour)
    
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
    
    (requiredcategory preentrance intronau)
    (requiredcategory staffintro idle)
    (requiredcategory prep1 distraction)
    (requiredcategory prep1 cognitivebehaviour)
    (requiredcategory prep2 distraction)
    (requiredcategory prep2 cognitivebehaviour)
    (requiredcategory tourniquet distraction)
    (requiredcategory tourniquet cognitivebehaviour)
    (requiredcategory iv distraction)
    (requiredcategory iv cognitivebehaviour)
    (requiredcategory tidy distraction)
    (requiredcategory tidy cognitivebehaviour)
    (requiredcategory goodbye idle)
    (requiredcategory postleave byenau)
        
    (anxietytest staffintro)
    
    (similar low low)
    (similar low medium)
    (similar medium medium)
    (similar medium low)
    (similar high high)
    (similar high medium)
    (similar medium high)
    
    (distractionstrength song1 medium)
    (distractionstrength song2 high)
    (distractionstrength dance1 high)
    (distractionstrength dance2 medium)
    (distractionstrength quiz medium)
    (distractionstrength leadmeditation low)
    (distractionstrength taichi medium)
    (distractionstrength intro low)
    (distractionstrength bye low)
    (distractionstrength idle1 low)
    (distractionstrength idle2 low)
    (distractionstrength idle3 low)
    
    (desiredstrength preentrance low)
    (desiredstrength staffintro low)
    (desiredstrength prep1 medium)
    (desiredstrength prep2 high)
    (desiredstrength tourniquet medium)
    (desiredstrength iv high)
    (desiredstrength tidy medium)
    (desiredstrength goodbye low)
    (desiredstrength postleave low)
    
  )
  (:goal
    (and
      (procedurestep)
      (procstage postleave)
    )
  )
)
