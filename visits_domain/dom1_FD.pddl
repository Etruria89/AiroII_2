(define (domain localization)

(:requirements :typing :durative-actions :numeric-fluents :negative-preconditions :action-costs :conditional-effects :equality :fluents )


(:types 	robot region 
)

(:predicates
		(robot_in ?v - robot ?r - region) (visited ?r - region )
	      
)

(:functions 
		(act-cost) (triggered ?from ?to - region) (dummy)
)

;;(:action goto_region
;;		:parameters 		(?v - robot ?from ?to - region)
;;		:precondition 		(robot_in ?v ?from)
;;	    :effect 	(and 	(not (robot_in ?v ?from))
;;							(robot_in ?v ?to))
;;)


(:durative-action localize
		:parameters 		(?v - robot ?from ?to - region)
		:duration 			(= ?duration 100)
		:condition 	(and 	(at start (robot_in ?v ?from))
							)
	    :effect 	(and 	(at start (not (robot_in ?v ?from))) 	
							(at start (increase (triggered ?from ?to) 1))
							(at end (robot_in ?v ?to)) 
							(at end (assign (triggered ?from ?to) 0)) 
							(at end (visited ?to)) 	
                			(at end (increase (act-cost) (dummy))))
)


)