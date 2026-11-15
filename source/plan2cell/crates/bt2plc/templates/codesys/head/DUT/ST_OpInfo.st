(*
 * ST_OpInfo
 *
 * One op of one robot (GVL_Cell.aOps). Op k of robot r is
 * GVL_Cell.aOps[GVL_Cell.anRobotFirstOp[r] + k - 1].
 *)

TYPE ST_OpInfo :
STRUCT
    nRobot              : INT;      (* robot index, 1-based *)
    nOp                 : INT;      (* op index of that robot, 1-based *)
    dwZoneMask          : DWORD;    (* REQUEST/GRANT mask: zone bits 0..30, bit 31 = permit *)
    nPredFirst          : INT;      (* permit preds: GVL_Cell.aPreds[nPredFirst .. nPredFirst + nPredCount - 1] *)
    nPredCount          : INT;      (* 0 = no permit predecessors *)
    tExpected           : TIME;     (* nominal duration *)
    tLimit              : TIME;     (* op watchdog (from the grant, or from the step change if mask = 0) *)
    tZoneDeadline       : TIME;     (* latest grant time since cycle start *)
    sName               : STRING(32);
END_STRUCT
END_TYPE
