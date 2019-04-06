;********************************************************
;*                                                      *
;*      actions_three.clp                               *
;*                                                      *
;*                                                      *
;*                      University of Mexico            *
;*                      Julio Cesar Cruz Estrda         *
;*                                                      *
;*                      28/03/2019                      *
;*                                                      *
;********************************************************

(defrule task_get_object_room
	?f <- (task ?plan get_object ?obj ?room ?step)
	?f1 <- (item (name ?obj)(type Objects))
        (item (type Room) (name ?room))
	?f2 <- (item (name finish_objetive))
	=>
	(retract ?f)
	(printout t "Try to find the object inside the room" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_grabed)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pgobjroom ?obj ?room ?step))
	(modify ?f1 (status nil))
	(modify ?f2 (status nil))
)

(defrule task_get_bag
	?f <- (task ?plan get_bag ?luggage ?step)
	?f1 <- (item (name finish_objetive))
	=>
	(retract ?f)
	(printout t "Get luggage and try to take it to some taxi or car" crlf)
	(assert (state (name ?plan) (number ?step)(duration 6000)))
	(assert (condition (conditional if) (arguments finish_objetive status finaly_grabed)(true-state (+ ?step 1))(false-state ?step)(name-scheduled ?plan)(state-number ?step)))
	(assert (task pget_bag ?step))
	(modify ?f1 (status nil))

)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;  SPLIT IN SUBTAREAS

(defrule plan_get_object_room
        ?goal <- (objetive gobjroom ?name ?obj ?room ?step)
        =>
        (retract ?goal)
        (printout t "Prueba Nuevo PLAN Get Object Task" crlf)
        (assert (plan (name ?name) (number 1)(actions ask_for ?obj ?room)(duration 6000)))
        (assert (plan (name ?name) (number 2)(actions review_room ?obj ?room)(actions_num_params 3 5)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions make_task ?name ?obj finded)(actions_num_params 4 4)(duration 6000)))
	(assert (plan (name ?name) (number 4)(actions move manipulator ?obj)(duration 6000)))
	(assert (plan (name ?name) (number 5)(actions update_status finish_objetive finaly_grabed)(duration 6000)))
	(assert (finish-planner ?name 100))
)

(defrule plan_get_bag
	?goal <- (objetive get_bag ?name ?step)
	=>
	(retract ?goal)
	(printout t "Prueba Nuevo PLAN Get bag" crlf)
	(assert (plan (name ?name) (number 1)(actions get_bag)(duration 6000)))
	(assert (plan (name ?name) (number 2)(actions update_status man went)(duration 6000)))
	(assert (plan (name ?name) (number 3)(actions update_status finish_objetive finaly_grabed)(duration 6000)))
	(assert (finish-planner ?name 2))
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; RULES BEFORE SPLIT IN SUBTAREAS


(defrule exe_scheduled-get-object-room 
        (state (name ?name) (number ?step)(status active)(duration ?time))
	(item (name ?robot)(zone ?zone))
        (name-scheduled ?name ?ini ?end)
        ?f1 <- (task pgobjroom ?obj ?room ?step)
        =>
        (retract ?f1)
        (assert (objetive gobjroom task_gobjroom ?obj ?room ?step))
)

(defrule exe_scheduled-get-bag
	(state (name ?name)(number ?step) (status active) (duration ?time))
	(item (name ?robot)(zone ?zone))
	(name-scheduled ?name ?ini ?end)
	?f1 <- (task pget_bag ?step)
	=>
	(retract ?f1)
	(assert (objetive get_bag task_get_bag ?step))
)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;