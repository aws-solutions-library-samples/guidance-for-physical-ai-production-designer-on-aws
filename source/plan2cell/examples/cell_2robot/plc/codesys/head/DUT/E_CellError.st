(*
 * E_CellError
 *
 * Fault codes of the cell (FB_Cell.eError), the robot drivers (FB_VassRobot.eError)
 * and the zone arbiter (FB_ZoneArbiter.eFault). ERR_NONE = no fault.
 *)

TYPE E_CellError :
(
    ERR_NONE                := 0,

    (* VASS bring-up and job handshake (FB_VassRobot) *)
    ERR_SAFETY_TIMEOUT      := 10,      (* SafeIn0.0 / SafeIn1.4 not reported *)
    ERR_DRIVES_TIMEOUT      := 11,      (* DrivesOn not reported *)
    ERR_EXT_TIMEOUT         := 12,      (* EXT mode not reported *)
    ERR_NOT_READY           := 13,      (* robot not Ready (or JobAck stuck) before a job *)
    ERR_ACK_TIMEOUT         := 14,      (* no JobAck after JobRequest *)
    ERR_ECHO_MISMATCH       := 15,      (* echoed job number <> requested job number *)
    ERR_START_TIMEOUT       := 16,      (* ProgRunning not reported after ProgStart *)
    ERR_ROBOT_STOPPED       := 17,      (* ProgRunning dropped without ProgFinished (T_STOP_DEBOUNCE) *)
    ERR_ROBOT_ERROR         := 18,      (* robot reports Error *)
    ERR_SAFETY_LOST         := 19,      (* safe status lost while up *)
    ERR_STOP_TIMEOUT        := 20,      (* robot still running after Stop + Cancel *)
    ERR_RESET_FAILED        := 21,      (* robot still in error after Reset *)
    ERR_DRIVES_LOST         := 22,      (* DrivesOn lost while up *)
    ERR_UNEXPECTED_RUN      := 23,      (* ProgRunning reported outside a job (off / bring-up / idle) *)

    (* zone protocol and supervision (FB_ZoneArbiter, FB_Cell) *)
    ERR_OP_WATCHDOG         := 30,      (* op took longer than aOps[].tLimit *)
    ERR_ZONE_WAIT_TIMEOUT   := 31,      (* zone wait past aOps[].tZoneDeadline since cycle start *)
    ERR_STEP_SEQUENCE       := 32,      (* op index did not advance by exactly +1, or 0 between two ops *)
    ERR_REQ_MISMATCH        := 33,      (* REQUEST <> mask of the reported op (debounced) *)
    ERR_TICKET_SKIPPED      := 34,      (* own ticket for another op, or zone consumed (debounced) *)
    ERR_PROGRAM_INCOMPLETE  := 35,      (* job done but last op index <> op count *)
    ERR_RELEASE_INVALID     := 36,      (* REQUEST dropped by a robot that is not running OK *)
    ERR_ROBOT_STALLED       := 37,      (* zone op without REQUEST and grant, or op index 0 too long *)
    ERR_UNEXPECTED_REQUEST  := 38,      (* holder raised bits it was not granted *)
    ERR_TICKETS_LEFT        := 39,      (* cycle done with unconsumed tickets *)
    ERR_HOME_FAILED         := 40,      (* blind homing did not reach the home pose *)
    ERR_HANDSHAKE           := 41,      (* op index changed while a grant was held (4-phase violated) *)
    ERR_ZONE_BYPASSED       := 42,      (* zone / permit op left without ever being granted *)

    (* operator *)
    ERR_OPERATOR_STOP       := 50       (* bStop *)
);
END_TYPE
