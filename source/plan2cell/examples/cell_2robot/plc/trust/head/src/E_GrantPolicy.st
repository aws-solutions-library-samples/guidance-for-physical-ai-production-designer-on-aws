(*
 * E_GrantPolicy
 *
 * Zone grant policy of FB_ZoneArbiter (GVL_Cell.GRANT_POLICY).
 *)

TYPE E_GrantPolicy :
(
    GP_PLANNED  := 0,       (* zone granted only to its next ticket holder (BT order) *)
    GP_FCFS     := 1        (* oldest request first - BT order NOT enforced *)
);
END_TYPE
