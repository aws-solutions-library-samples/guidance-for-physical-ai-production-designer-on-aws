(*
 * E_RobotState
 *
 * States of the VASS driver FB_VassRobot (one robot).
 *)

TYPE E_RobotState :
(
    RS_OFF                  := 0,       (* outputs off, waiting for bPowerOn *)
    RS_SAFETY_ENABLE        := 1,       (* SafeOut0/1 set, waiting for safe status *)
    RS_DRIVES_ON            := 2,       (* DrivesOn requested *)
    RS_MODE_EXT             := 3,       (* EXT mode requested *)
    RS_IDLE                 := 10,      (* up, no job *)
    RS_JOB_WAIT_READY       := 20,      (* waiting for Ready and no JobAck *)
    RS_JOB_SEND             := 21,      (* job number + JobRequest, waiting for JobAck *)
    RS_JOB_START            := 22,      (* ProgStart pulse *)
    RS_JOB_WAIT_RUNNING     := 23,      (* waiting for ProgRunning (or ProgFinished) *)
    RS_JOB_RUNNING          := 24,      (* program running *)
    RS_JOB_DONE             := 25,      (* ProgFinished seen, bJobDone set *)
    RS_STOPPING             := 30,      (* Stop + Cancel held *)
    RS_RESET                := 40,      (* Reset held *)
    RS_ERROR                := 99       (* latched until bCmdReset *)
);
END_TYPE
