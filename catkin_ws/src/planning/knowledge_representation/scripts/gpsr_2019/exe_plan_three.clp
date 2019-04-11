;********************************************************
;*                                                      *
;*      actions_three.clp                               *
;*                                                      *
;*                                                      *
;*                      University of Mexico            *
;*                      Julio Cesar Cruz Estrda         *
;*                                                      *
;*                      03/04/2019                      *
;*                                                      *
;********************************************************


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;  get bag task
(defrule exe-plan-task-get-bag 
	(plan (name ?name) (number ?num-pln) (status active) (actions get_bag) (duration ?t))
	=>
	(bind ?command (str-cat "bag"))
	(assert (send-blackboard ACT-PLN cmd_get_bag ?command ?t 4))
)

(defrule exe-plan-task-geted-bag 
	?f <- (received ?sender command cmd_get_bag bag 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions get_bag))
        ?f2 <- (Arm (name right))
	?f3 <- (item (name bag))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
	(modify ?f2 (status ready) (grasp bag))
	(modify ?f3 (status grabed))
)

(defrule exe-plan-task-no-geted-bag 
	?f <- (received ?sender command cmd_get_bag bag 0)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions get_bag))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
)
;;;;;;;;;;;;;;;;;;;;; follow to the taxi, uber, cab
(defrule exe-plan-task-follow-task
	(plan (name ?name) (number ?num-pln) (status active) (actions follow_to_taxi man ?place) (duration ?t))
	=>
	(bind ?command (str-cat "" ?place ""))
	(assert (send-blackboard ACT-PLN cmd_follow_to_taxi ?command ?t 4))
)

(defrule exe-plan-task-followed-to-taxi 
	?f <- (received ?sender command cmd_follow_to_taxi ?place 1)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions follow_to_taxi man ?place))
	?f2 <- (item (name man))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
	(modify ?f2 (status followed))
)

(defrule exe-plan-no-followed-to-taxi
	?f <- (received ?sender command cmd_follow_to_taxi ?place 0)
	?f1 <- (plan (name ?name) (number ?num-pln) (status active) (actions follow_to_taxi man ?place))
	=>
	(retract ?f)
	(modify ?f1 (status accomplished))
)

;;;;;;;;;;;;;;;;;;;;;;; get the abspos object
(defrule exe-plan-task-get-pos-object
	(plan (name ?name) (number ?num-pln) (status active) (actions find-pos-object ?place ?pos)(duration ?t))
	=>
	(bind ?command (str-cat "pose "  ?place " " ?pos ""))
	(assert (send-blackboard ACT-PLN find_object ?command ?t 4))
)
(defrule exe-plan-geted-pos-object 
        ?f <-  (received ?sender command find_object ?block1 ?x ?y ?z ?arm 1)
 	    ?f1 <- (item (name ?block1))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-pos-object ?place ?pos))
	    ?f3 <- (Arm (name ?arm))
	?f4 <- (item (name object))
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
        (modify ?f3 (status ready) (grasp ?block1))
	(modify ?f1 (pose ?x ?y ?z) (status finded));;;; modified for verify arm task
	(modify ?f4 (image ?block1)(status finded))
)

(defrule exe-plan-no-geted-pos-object 
        ?f <-  (received ?sender command find_object ?block1 ?x ?y ?z ?arm 0)
        ?f1 <- (item (name ?block1))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions find-pos-object ?place ?pos))
        =>
        (retract ?f)
	(modify ?f2 (status accomplished)) ;performance for IROS
        ;(modify ?f2 (status active))//normal performance gpsr and eegpsr
)
;;;;;;;;;;;;;;;;;;;;;;;;;; grasp object

(defrule exe-plan-grasp-pos-object
	(plan (name ?name) (number ?num-pln) (status active) (actions move ?actuator)(duration ?t))
	(item (name object) (image ?obj))
	(item (name ?obj) (pose ?x ?y ?z))
        (Arm (name ?arm) (status ready)(bandera ?id) (grasp ?obj))
	=>
        (bind ?command (str-cat "" ?obj " " ?x " " ?y " " ?z " " ?id ""))
        (assert (send-blackboard ACT-PLN move_actuator ?command ?t 4))
)

(defrule exe-plan-grasped-pos-object 
        ?f <-  (received ?sender command move_actuator ?object ?x ?y ?z ?id 1)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions move ?actuator))
	?f3 <- (item (name robot));;;;;;;;;; T1 test for quit grasp object subtask
        =>
        (retract ?f)
        (modify ?f2 (status accomplished))
	(modify ?f3 (hands ?object));;;;; T1 test
	(modify ?f1 (status grabed));;;;;; T1 test
        ;(retract ?f3)
)

(defrule exe-plan-no-grasped-pos-object
        ?f <-  (received ?sender command move_actuator ?object ?x ?y ?z ?id 0)
        ?f1 <- (item (name ?object))
        ?f2 <- (plan (name ?name) (number ?num-pln)(status active)(actions move ?actuator))
	?f3 <- (Arm (grasp ?object))
        =>
        (retract ?f)
	(modify ?f2 (status accomplished))
	(modify ?f3 (status nil) (grasp nil))
)


;;;;;;;;;;;;;;;;;;;;;;;;;;
