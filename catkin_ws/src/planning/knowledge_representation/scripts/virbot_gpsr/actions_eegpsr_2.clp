;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;					;;;
;;;	University of Mexico		;;;
;;;	18/Abril/2018			;;;
;;;	Julio Cesar Cruz Estrada	;;;
;;;					;;;
;;;	EEGPSR				;;;
;;;	category 2			;;;
;;;					;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;#tell me how many $people there are in the $room
;#tell me how many $peopleR in the $room
;#tell me how many $ppl in the $room are $peopleDsc
(defrule task_find_how_many_people_dsc
	?f <- (task ?plan find_how_many_people ?ppl ?peopleDsc ?place ?step)
	?f1 <- (item (name ?ppl))
	=>
	(retract ?f)
	(printout t "How many people task" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments ?ppl status finded)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (cd-task (cd pfhmp) (actor robot)(obj ?ppl)(from ?peopleDsc)(to ?place)(name-scheduled ?plan)(state-number ?step)))
	;;;;;test reiniciar status del parametro
	(modify ?f1 (status nil))
)

(defrule task_find_how_many_people 
	?f <- (task ?plan find_how_many_people ?ppl ?place ?step)
	?f1 <- (item (name ?ppl))
	=>
	(retract ?f)
	(printout t "How many people task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments ?ppl status finded) (true-state (+ ?step 1))(false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pfhmp ?ppl ?place ?step))
	(modify ?f1 (status nil))
)

;#offer something to (eat | drink) to all the $people in the $room
(defrule task_offer_eat_drink 
	?f <- (task ?plan offer_eat_drink ?ppl ?eatdrink ?place ?step)
	?f1 <- (item (name offer))
	=>
	(retract ?f)
	(printout t "Offer something to eat or drink to people task" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments offer status final_offer)(true-state (+ ?step 1)) (false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task poffer ?ppl ?eatdrink ?place ?step))
	(modify ?f1 (status nil))
)

(defrule task_offer_eat_drink_dsc
	?f <- (task ?plan offer_eat_drink ?ppl ?peopleDsc ?eatdrink ?place ?step)
	?f1 <- (item (name offer))
	=>
	(retract ?f)
	(printout t "Offer something to eat or drink to people task" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments offer status final_offer)(true-state (+ ?step 1)) (false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task poffer ?ppl ?peopleDsc ?eatdrink ?place ?step))
	(modify ?f1 (status nil))
)

;#bring the {objetc} to the $person in the $room
(defrule task_bring_obj_to_prsn
	?f <- (task ?plan bring_obj_to_prsn ?ppl ?peopleDsc ?obj ?place ?step)
	?f1 <- (item (name ?obj))
	=>
	(retract ?f)
	(printout t "Bring object to person task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments ?obj status droped) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pbringotp ?ppl ?peopleDsc ?obj ?place ?step))
	(modify ?f1 (status nil))
)

;#$greet the $person in the $room
(defrule task_greet_person_dsc
	?f <- (task ?plan greet_person ?ppl ?peopleDsc ?place ?step)
	?f1 <- (item (name question_1))
	=>
	(retract ?f)
	(printout t "Greet the person at the room task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments question_1 status ask) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pgreet ?ppl ?peopleDsc ?place ?step))
	(modify ?f1 (status nil))
)

(defrule task_greet_person_outfit 
	?f <- (task ?plan greet_peson ?ppl ?color ?outfit ?place ?step)
	?f1 <- (item (name question_1))
	=>
	(retract ?f)
	(printout t "Greet the person at the room task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments question_1 status ask) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pgreet_outfit ?ppl ?color ?outfit ?place ?step))
	(modify ?f1 (status nil))
)


        ;#$vbfollow the $fgwhor
        ;#$vbfollow the $fgwho $fbriefing
(defrule task_follow_man
	?f <- (task ?plan follow_person ?ppl ?place ?step)
	?f1 <- (item (name man))
	=>
	(retract ?f)
	(printout t "Follow the person in the room task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments man status followed) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pfollow_person ?ppl ?place ?step))
	(modify ?f1 (status nil))
)

(defrule task_follow_person
	?f <- (task ?plan follow_person ?ppl ?peopleDsc ?place ?step)
	?f1 <- (item (name man))
	=>
	(retract ?f)
	(printout t "Follow the person in the room task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments man status followed) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pfollow_person ?ppl ?peopleDsc ?place ?step))
	(modify ?f1 (status nil))
)

        ;#$vbfollow the $fgwhor
(defrule task_follow_person_outfit
	?f <- (task ?plan follow_person ?ppl ?color ?outfit ?place ?step)
	?f1 <- (item (name man))
	=>
	(retract ?f)
	(printout t "Follow person to room task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments man status followed) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pfollow_person_outfit ?ppl ?color ?outfit ?place ?step))
	(modify ?f1 (status nil))
)

        ;#$vbguide the $fgwhor to the (exit | {room 2})
(defrule task_guide_man
	?f <- (task ?plan guide_person ?ppl ?place1 ?place2 ?step)
	?f1 <- (item (name man_guide))
	=>
	(retract ?f)
	(printout t "Guide person from room 1 to room 2" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments man_guide status followed) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pguide_person ?ppl ?place1 ?place2 ?step))
	(modify ?f1 (status nil))
)

(defrule task_guide_person
	?f <- (task ?plan guide_person ?ppl ?peopleDsc ?place1 ?place2 ?step)
	?f1 <- (item (name man_guide))
	=>
	(retract ?f)
	(printout t "Guide person from room 1 to room 2 task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments man_guide status followed) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pguide_person ?ppl ?peopleDsc ?place1 ?place2 ?step))
	(modify ?f1 (status nil))
)

(defrule task_guide_person_outfit
	?f <- (task ?plan guide_person ?ppl ?color ?outfit ?place1 ?place2 ?step)
	?f1 <- (item (name man_guide))
	=>
	(retract ?f)
	(printout t "Guide person from room 1 to room 2 task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments man_guide status followed) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pguide_person_outfit ?ppl ?color ?outfit ?place1 ?place2 ?step))
	(modify ?f1 (status nil))
)

(defrule task_describe_person_to_me
	?f <- (task ?plan describe_person ?posture ?place ?step)
	?f1 <- (item (name question_2))
	?f2 <- (item (name ?posture))
	=>
	(retract ?f)
	(printout t "Describe the person at the room task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments question_2 status ask) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pdescribe_person_to_me ?posture ?place ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_describe_person
	?f <- (task ?plan describe_person ?posture ?place1 ?place2 ?step)
	?f1 <- (item (name question_2))
	?f2 <- (item (name ?posture))
	=>
	(retract ?f)
	(printout t "Describe the person at the room task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments question_2 status ask) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pdescribe_person ?posture ?place1 ?place2 ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

;$vbmeet {name} at the {beacon 1}
(defrule task_remind_person
	?f <- (task ?plan remind_person ?person ?place ?step)
	?f1 <- (item (name ?person))
	=>
	(retract ?f)
	(printout t "Conoce y recuerda a una person task" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments ?person status reminded) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task premind_person ?person ?place ?step))
	(modify ?f1 (status remind))
)

(defrule task_greet_known_person
	?f <- (task ?plan greet_known_person ?place ?step)
	?f1 <- (item (name ?person) (status remind))
	=>
	(retract ?f)
	(printout t "Busca y saluda a una persona que conoces" crlf)
	(assert (state (name ?plan) (number ?step) (duration 6000)))
	(assert (condition (conditional if) (arguments ?person status greeted) (true-state (+ ?step 1)) (false-state ?step) (name-scheduled ?plan) (state-number ?step)))
	(assert (task pgreet_known_person ?place ?step))
) 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;; subtask

(defrule plan_find_how_many_people
	?goal <- (objetive find_how_many_people ?name ?ppl ?place ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Find how many people task" crlf)
	(bind ?speech (str-cat "I tried to find people"))
	(assert (plan (name ?name) (number 1)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions find-many-people ?ppl ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions go_to_place current_loc) (duration 6000)))
	(assert (plan (name ?name) (number 4)(actions speech-anything ?speech) (duration 6000)))
	(assert (plan (name ?name) (number 5)(actions update_status ?ppl finded) (duration 6000)))
	(assert (finish-planner ?name 5))
)

(defrule plan_find_how_many_people_dsc
        ?goal <- (objetive find_how_many_people ?name ?ppl ?peopleDsc ?place ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Find how many people Task" crlf)
	(bind ?speech(str-cat "I tried to find people"))
	(assert (plan (name ?name) (number 1)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions find-many-people  ?ppl ?peopleDsc ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions go_to_place current_loc)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions speech-anything ?speech)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions update_status ?ppl finded) (duration 6000)))
	(assert (finish-planner ?name 5))
)

(defrule plan_offer_eat_drink
	?goal <- (objetive offer_eat_drink ?name ?ppl ?eatdrink ?place ?step)
	=>
	(retract ?goal)
	(printout t "Preueba Nuevo PLAN Offer something to drink")
	(assert (plan (name ?name) (number 1) (actions go_to_place ?place) (duration 6000)))
	(assert (plan (name ?name) (number 2) (actions get_amount_people ?place) (duration 6000)))
	(assert (plan (name ?name) (number 3) (actions ask_and_offer ?ppl ?eatdrink ?place) (duration 6000)))
	(assert (finish-planner ?name 3))
)

(defrule plan_offer_eat_drink_dsc
	?goal <- (objetive offer_eat_drink ?name ?ppl ?peopleDsc ?eatdrink ?place ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Offer something to drink")
	(assert (plan (name ?name) (number 1) (actions go_to_place ?place) (duration 6000)))
	(assert (plan (name ?name) (number 2) (actions get_amount_people ?place)))
	(assert (plan (name ?name) (number 3) (actions ask_and_offer ?ppl ?peopleDsc ?eatdrink ?place)))
	(assert (finish-planner ?name 3))
)

(defrule plan_bring-obj-to-person
	?goal <- (objetive bring_obj_to_prsn ?name ?ppl ?peopleDsc ?obj ?place ?step)
	=>
	(retract ?goal)
	(assert (plan (name ?name) (number 1)(actions go_to ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions attend ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions find-object ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions move manipulator ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions find-endurance-person ?ppl ?peopleDsc ?place)(duration 6000)))
	(assert (plan (name ?name) (number 7)(actions drop person ?obj)(duration 6000)))
	(assert (finish-planner ?name 7))
)

(defrule plan_greet-person
	?goal <- (objetive greet_person ?name ?ppl ?peopleDsc ?place ?step)
	=>
	(retract ?goal)
	(assert (plan (name ?name) (number 1)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions find-endurance-person ?ppl ?peopleDsc ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions answer_question question_1 introduce_yourself)(duration 6000)))
	(assert (finish-planner ?name 3))
)

(defrule plan_greet-person-outfit
	?goal <- (objetive greet_person_outfit ?name ?ppl ?color ?outfit ?place ?step)
	=>
	(retract ?goal)
	(assert (plan (name ?name) (number 1)(actions go_to_pace ?place)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions find-endurance-person ?ppl ?color ?outfit ?place)))
	(assert (plan (name ?name) (number 3)(actions answer_question question_1 introduce_yourself)(duration 6000)))
	(assert (finish-planner ?name 3))
)

(defrule plan_follow_man
	?goal <- (objetive follow_man ?name ?ppl ?place ?step)
	=>
	(retract ?goal)
	(assert (plan (name ?name) (number 1)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions find-person person ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions find-object-man man no_location)(duration 6000)))
	(assert (finish-planner ?name 3))
)

(defrule plan_follow_person
	?goal <- (objetive follow_person ?name ?ppl ?peopleDsc ?place ?step)
	=>
	(retract ?goal)
	(assert (plan (name ?name) (number 1)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions find-endurance-person ?ppl ?peopleDsc ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions find-object-man man no_location)(duration 6000)))
	(assert (finish-planner ?name 3))
)

(defrule plan_follow_person_outfit
	?goal <- (objetive follow_person_outfit ?name ?ppl ?color ?outfit ?place ?step)
	=>
	(retract ?goal)
	(assert (plan (name ?name) (number 1)(actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions find-endurance-person ?ppl ?color ?outfit ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions find-object-man man no_location)(duration 6000)))
	(assert (finish-planner ?name 3))
)

(defrule plan_guide_man
	?goal <- (objetive guide_man ?name ?ppl ?place1 ?place2 ?step)
	=>
	(retract ?goal)
	(assert (plan (name ?name) (number 1)(actions go_to_place ?place1) (duration 6000)))
	(assert (plan (name ?name) (number 2)(actions find-person person ?place1) (duration 6000)))
	(assert (plan (name ?name) (number 3)(actions find-object-man man_guide ?place2) (duration 6000)))
	(assert (finish-planner ?name 3))
)

(defrule plan_guide-person
	?goal <- (objetive guide_person ?name ?ppl ?peopleDsc ?place1 ?place2 ?step)
	=>
	(retract ?goal)
	(assert (plan (name ?name) (number 1)(actions go_to_place ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions find-endurance-person ?ppl ?peopleDsc ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions find-object-man man_guide ?place2)))
	(assert (finish-planner ?name 3))
)

(defrule plan_guide-person-outfit
	?goal <- (objetive guide_person_outfit ?name ?ppl ?color ?outfit ?place1 ?place2 ?step)
	=>
	(retract ?goal)
	(assert (plan (name ?name) (number 1)(actions go_to_place ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions find-endurance-person ?ppl ?color ?outfit ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions find-object-man man_guide ?place2)(duration 6000)))
	(assert (finish-planner ?name 3))
)

(defrule plan_describe-person-to-me
	?goal <- (objetive describe_person ?name ?posture ?place ?step)
	=>
	(retract ?goal)
	(assert (plan (name ?name) (number 1)(actions go_to_place ?place)(duration 6000)))
        (assert (plan (name ?name) (number 2)(actions gesture_person ?posture ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions scan_person ?posture ?place)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions go_to_place current_loc)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions answer_question question_2 introduce_yourself)(duration 6000)))
	(assert (finish-planner ?name 5))	
)

(defrule plan_describe-person
	?goal <- (objetive describe_person ?name ?posture ?place1 ?place2 ?step)
	=>
	(retract ?goal)
	(assert (plan (name ?name) (number 1)(actions go_to_place ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions gesture_person ?posture ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions scan_person ?posture ?place1)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions go_to_place ?place2)))
	(assert (plan (name ?name) (number 5)(actions find-person person ?place2)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions answer_question question_2 introduce_yourself)(duration 6000)))
	(assert (finish-planner ?name 6))
)

(defrule plan_remind-person
	?goal <- (objetive remind_person ?name ?person ?place ?step)
	=>
	(retract ?goal)
	(bind ?speech(str-cat "Hello, please look at me, I will try to remember you"))
	(bind ?speech_1(str-cat "thank you very much, Now I can remember you"))
	(assert (plan (name ?name) (number 1) (actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 2) (actions find-person person ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3) (actions speech-anything ?speech)(duration 6000)))
	(assert (plan (name ?name) (number 4) (actions remind_person ?person ?place) (duration 6000)))
	(assert (plan (name ?name) (number 5) (actions speech-anything ?speech_1) (duration 6000)))
	(assert (plan (name ?name) (number 6) (actions update_status ?person reminded) (duration 6000)))
	(assert (finish-planner ?name 6))
)

(defrule plan_greet-known-person
	?goal <- (objetive greet_known_person ?name ?person ?place ?step)
	=>
	(retract ?goal)
	(assert (plan (name ?name) (number 1) (actions set_plan_status ?name) (actions_num_params 3 4) (duration 6000)))
	(assert (plan (name ?name) (number 2) (actions go_to_place ?place)(duration 6000)))
	(assert (plan (name ?name) (number 3) (actions find-person person ?place) (duration 6000)))
	(assert (plan (name ?name) (number 4) (actions find-reminded-person ?person ?place) (duration 6000)))
	(assert (plan (name ?name) (number 5) (actions repeat_task ?name ?person greet) (actions_num_params 3 4 1 3) (duration 6000)))
	;(assert (plan (name ?name) (number 5) (actions  )))
	(assert (plan (name ?name) (number 6) (actions set_plan_status ?name) (actions_num_params 3 4) (duration 6000)))
	(assert (plan (name ?name) (number 7) (actions update_status ?person greeted) (duration 6000)))
	(assert (finish-planner ?name 7))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;; before split tasks

(defrule exe_find-how-many-people
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pfhmp ?ppl ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive find_how_many_people task_find_how_many_people ?ppl ?place ?step))
)

(defrule exe_find-how-many-people_dsc
        (state (name ?name) (number ?step)(status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (cd-task (cd pfhmp) (actor ?robot)(obj ?ppl)(from ?peopleDsc)(to ?place)(name-scheduled ?name)(state-number ?step))
        =>
        (retract ?f1)
        (assert (objetive find_how_many_people task_find_how_many_people ?ppl ?peopleDsc ?place ?step))
)

(defrule exe_offer_eat_drink
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task poffer ?ppl ?eatdrink ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive offer_eat_drink task_offer_eat_drink ?ppl ?eatdrink ?place ?step))
)

(defrule exe_offer_eat_drink_dsc
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task poffer ?ppl ?peopleDsc ?eatdrink ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive offer_eat_drink task_offer_eat_drink ?ppl ?peopleDsc ?eatdrink ?place ?step))
)

(defrule exe_bring-obj-to-persn 
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pbringotp ?ppl ?peopleDsc ?obj ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive bring_obj_to_prsn task_bring_obj_to_prsn ?ppl ?peopleDsc ?obj ?place ?step))
)

(defrule exe_greet-person
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pgreet ?ppl ?peopleDsc ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive greet_person task_greet_person ?ppl ?peopleDsc ?place ?step))
)

(defrule exe_greet-person-outfit
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pgreet_outfit ?ppl ?color ?outfit ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive greet_person_outfit task_greet_person_outfit ?ppl ?color ?outfit ?place ?step))
)

(defrule exe_follow-man
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pfollow_person ?ppl ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive follow_man task_follow_man ?ppl ?place ?step))
)

(defrule exe_follow-person
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pfollow_person ?ppl ?peopleDsc ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive follow_person task_follow_person ?ppl ?peopleDsc ?place ?step))
)

(defrule exe_follow-person-outfit
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pfollow_person_outfit ?ppl ?color ?outfit ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive follow_person_outfit task_follow_person_outfit ?ppl ?color ?outfit ?place ?step))
)

(defrule exe_guide-man
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pguide_person ?ppl ?place1 ?place2 ?step)
	=>
	(retract ?f1)
	(assert (objetive guide_man task_guide_man ?ppl ?place1 ?place2 ?step))
)

(defrule exe_guide-person
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pguide_person ?ppl ?peopleDsc ?place1 ?place2 ?step)
	=>
	(retract ?f1)
	(assert (objetive guide_person task_guide_person ?ppl ?peopleDsc ?place1 ?place2 ?step))
)

(defrule exe_guide_person_outfit
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pguide_person_outfit ?ppl ?color ?outfit ?place1 ?place2 ?step)
	=>
	(retract ?f1)
	(assert (objetive guide_person_outfit task_guide_person_outfit ?ppl ?color ?outfit ?place1 ?place2 ?step))
)

(defrule exe_describe-person-to-me
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pdescribe_person_to_me ?posture ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive describe_person task_describe_person ?posture ?place ?step))
)

(defrule exe_describe-person
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pdescribe_person ?posture ?place1 ?place2 ?step)
	=>
	(retract ?f1)
	(assert (objetive describe_person task_describe_person ?posture ?place1 ?place2 ?step))
)

(defrule exe_remind-person
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task premind_person ?person ?place ?step)
	=>
	(retract ?f1)
	(assert (objetive remind_person task_remind_person ?person ?place ?step))
)

(defrule exe_greet-known-person
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pgreet_known_person ?place ?step)
	(item (name ?person) (status reminded))
	=>
	(retract ?f1)
	(assert (objetive greet_known_person task_greet_known_person ?person ?place ?step))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;