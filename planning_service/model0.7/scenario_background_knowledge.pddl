(define (problem proc1)
  (:domain ivplacementsensing)
  (:objects 

    intro bye ivdescription song1 song2 dance1 dance2 leadmeditation taichi quiz magic - activity
    ;mitigation1 mitigation2 - activity
    introstep preprocedure procedure debrief end - procstep
    distraction cognitivebehaviour proceduredescription intronau reward byenau - activitytype
    ;mitigationaction - activitytype
    low medium high vhigh - level
  )
  (:init
    
    
    (stepproc introstep preprocedure)
    (stepproc preprocedure procedure)
    (stepproc procedure debrief)
    (stepproc debrief end)

    ;(anxietymitigating mitigationaction)
    
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
    (activitycategory magic distraction)
    
    ;(activitycategory mitigation1 mitigationaction)
    ;(activitycategory mitigation2 mitigationaction)

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
        
;    (anxietytest preprocedure)
;    (anxietytest procedure)
    
    (similar low low)
    (similar low medium)
    (similar medium medium)
    (similar medium low)
    (similar high high)
    (similar high medium)
    (similar medium high)
    (similar vhigh vhigh)
    (similar vhigh high)
    (similar high vhigh)
    
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
    (distractionstrengthyoung magic vhigh)
    
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
    (distractionstrengthold magic vhigh)
    
    (desiredstrength introstep low)
    (desiredstrength preprocedure high)
    (desiredstrength procedure high)
    (desiredstrength debrief medium)
    (desiredstrength end low)
    
    (desiredstrengthmitigate introstep medium)
    (desiredstrengthmitigate preprocedure vhigh)
    (desiredstrengthmitigate procedure vhigh)
    (desiredstrengthmitigate debrief high)
    (desiredstrengthmitigate end medium)
   
    (same intro intro)
    (same bye bye)
    (same ivdescription ivdescription)
    (same song1 song1)
    (same song2 song2)
    (same dance1 dance1)
    (same dance2 dance2)
    (same leadmeditation leadmeditation)
    (same taichi taichi)
    (same quiz quiz)
    (same magic magic)
    ;(same mitigation1 mitigation1)
    ;(same mitigation2 mitigation2)
  )
  (:goal
    (and
      (procedurestep)

      (procstage end)

    )
  )
)
