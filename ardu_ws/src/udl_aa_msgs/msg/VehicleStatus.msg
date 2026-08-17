# Vehicle -> flight commander. Published every tick without exception: it is a
# heartbeat, not a notification, so the commander stays aware of current state
# even on ticks where nothing was submitted and nothing happened.
#
# cmd_status and command are independent. cmd_status describes what became of
# the commander's submission; command describes what the vehicle actually
# dispatched. Safety firing with nothing submitted is IDLE *with* a command.

string ACCEPTED=ACCEPTED
string REJECTED=REJECTED
string IDLE=IDLE

std_msgs/Header header

string   cmd_status   # ACCEPTED / REJECTED / IDLE - the fate of the submission
string[] cond         # every condition with something to say this tick; empty when clean
string   command      # the command dispatched, or empty if none was
uint32   token        # echoed from VehicleCommand; 0 when nothing was submitted

# Reading the combinations:
#   ACCEPTED + command       - the submission was dispatched
#   REJECTED + no command    - blocked by an axis; cond says which
#   REJECTED + command       - valid, but safety outranked it; command is safety's
#   IDLE     + no command    - nothing submitted, nothing to do
#   IDLE     + command       - nothing submitted, safety acted on its own
