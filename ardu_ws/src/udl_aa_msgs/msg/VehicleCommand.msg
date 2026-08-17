# Flight commander -> vehicle. One command per message.
#
# Parameters travel as parallel arrays rather than typed per-command fields:
# every parameter of every command in the stack is a float, so this stays
# general without a message type per command. fs_adapter zips the two arrays
# into the dict the fsm already expects.
#
# The cost of that generality is that the type system cannot catch a mismatched
# array length, an unknown command, or a misspelled parameter name. fs_adapter
# desk-rejects all of those in its subscription callback, before the command
# reaches the fsm - so they come back as cmd_status=REJECTED with
# cond=['MALFORMED'], never as a SANITY failure, which would wrongly imply the
# values were merely out of bounds.

std_msgs/Header header

string    sequence       # issuing sequence or program; identity, not addressing
uint32    token          # identifies this submission, echoed back in VehicleStatus
string    command        # e.g. TAKEOFF, GOTO_LOCAL, SET_VELOCITY

string[]  param_names    # must match param_values in length, and must be
float64[] param_values   # exactly the parameter set the command expects
