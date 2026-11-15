(*
 * ST_RobotIn
 *
 * Copy of one robot's input image (robot -> PLC), filled by PLC_PRG from
 * GVL_IO.bySafeIn0_<tag>..bySafeIn7_<tag> and GVL_IO.aVassInputData_<tag>.
 *)

TYPE ST_RobotIn :
STRUCT
    abySafeIn           : ARRAY[0..7] OF BYTE;      (* PROFIsafe FDIO64 safe status *)
    aVassIn             : ARRAY[0..31] OF BYTE;     (* VASS DIO256 status telegram *)
END_STRUCT
END_TYPE
