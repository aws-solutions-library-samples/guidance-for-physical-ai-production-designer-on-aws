(*
 * ST_Ticket
 *
 * One (robot, op) reference: a zone ticket in GVL_Cell.aTickets or a permit
 * predecessor in GVL_Cell.aPreds. Both indices are 1-based; 0 = none (dummy row).
 *)

TYPE ST_Ticket :
STRUCT
    nRobot              : INT;      (* robot index 1..GVL_Cell.N_ROBOTS *)
    nOp                 : INT;      (* op index of that robot 1..anRobotOpCount[nRobot] *)
END_STRUCT
END_TYPE
