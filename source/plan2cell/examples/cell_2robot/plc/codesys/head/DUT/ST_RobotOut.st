(*
 * ST_RobotOut
 *
 * One robot's output image (PLC -> robot), copied by PLC_PRG to
 * GVL_IO.bySafeOut0_<tag>..bySafeOut7_<tag> and GVL_IO.aVassOutputData_<tag>.
 *)

TYPE ST_RobotOut :
STRUCT
    abySafeOut          : ARRAY[0..7] OF BYTE;      (* PROFIsafe FDIO64 safe commands *)
    aVassOut            : ARRAY[0..31] OF BYTE;     (* VASS DIO256 command telegram *)
END_STRUCT
END_TYPE
