;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;					;;;
;;;	University of Mexico		;;;
;;;	10/Enero/2010			;;;
;;;	Julio Cesar Cruz Estrada	;;;
;;;					;;;
;;;	Serving Drinks			;;;
;;;					;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule task_serving_drinks
	?f <- (task ?plan serving_drinks ?step)
	?f1 <- (item (name finish_objetive))
	=>
	(retract ?f)
	(printout t "Serving dring test")
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finish_test)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pserving_drinks ?step))
	(modify ?f1 status(nil))	
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule plan_serving_drinks
	?goal <- (objetive serving_drinks ?name ?step)
	=>
	(retract ?goal)
	(printout  "Nuevo plan Serving Drinks")
	(bind ?name (str-cat "Hello_what_is_your_name"))
	(bind ?remember (str-cat "Please look at me, I try to remember you"))
	(bind ?confirmation (str-cat "Do_you_want_some_to_drink"))
	(assert (plan (name ?name) (number 1)(actions set_plan_status ?name)(actions_num_params 2 4)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions set_param_in_plan ?person)(actions_num_params 3 4 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions go_to_place dummy_place)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions find-person specific ?person dummy_place)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions repeat_task ?name ?person went) (actions_num_params 2 4 1 3)(duration 6000)))
	(assert (plan (name ?name) (number 6)(actions set_plan_status ?name)(actions_num_params 2 4) (duration 6000)))
	(assert (plan (name ?name) (number 7)(actions update_status finish_objetive finded_in_some_room) (duration 6000)))
	(assert (finish-planner ?name 7))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule exe_serving_drinks
	(state (name ?name) (number ?step) (status active) (duration ?time))
	(item (name ?robot) (zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pserving_drinks ?step)
	=>
	(retract ?f1)
	(assert (objetive serving_drinks task_serving_drinks ?step))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
