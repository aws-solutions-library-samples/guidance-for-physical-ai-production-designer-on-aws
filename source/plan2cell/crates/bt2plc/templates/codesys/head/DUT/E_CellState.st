(*
 * E_CellState
 *
 * States of the cell controller FB_Cell.
 *)

TYPE E_CellState :
(
    CS_INIT                 := 0,       (* robots off *)
    CS_BRINGUP              := 10,      (* all robots: SAFETY_ENABLE -> DRIVES_ON -> MODE_EXT -> IDLE *)
    CS_HOMING_SELECT        := 20,      (* pick robot(s) not at home *)
    CS_HOMING_RUN           := 21,      (* blind homing with HOME_FOLGE *)
    CS_READY                := 30,      (* all robots at home, waiting for start / auto-run *)
    CS_START_JOBS           := 40,      (* production Folge commanded on every active robot *)
    CS_RUNNING              := 50,      (* robots run autonomously, zones arbitrated *)
    CS_CYCLE_DONE           := 60,      (* pause T_CYCLE_PAUSE before the next cycle *)
    CS_ABORTING             := 80,      (* grants dropped, Stop + Cancel to every busy robot *)
    CS_RECOVERY             := 85,      (* after an abort mid-cycle, or a start-up with an active robot
                                           away from home: wait for bRecoveryConfirm *)
    CS_RESETTING            := 90,      (* Reset to robots in error (an operator stop cancels it) *)
    CS_ERROR                := 99       (* latched until bReset *)
);
END_TYPE
