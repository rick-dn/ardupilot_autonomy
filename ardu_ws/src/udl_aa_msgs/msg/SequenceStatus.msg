# Sequence node -> commander. What is running, and how it ended.
#
# Published every tick without exception, including when nothing is running: it
# is a heartbeat, not a notification. The commander needs "nothing is running"
# as a positive fact - its rule is to deny a new sequence while one is live,
# unless that sequence is `abort` or a higher-priority safety one - and an
# event-only topic cannot tell it that after a dropped message.
#
# The token is how the commander knows the running sequence is the one it asked
# for. It mints one per request and this echoes it back, which distinguishes
# "my request is now running" from a stale status, and tells a sequence that
# completed from one still going.
#
# There is no feedback on phases or progress. A sequence owns its phases; the
# commander only ever needs to know whether something holds the vehicle and at
# what priority, so reporting more would be inventing a contract nobody reads.

std_msgs/Header header

string RUNNING=RUNNING
string COMPLETE=COMPLETE
string ABORT=ABORT

string sequence   # registry name, or empty when nothing is running
string state      # RUNNING / COMPLETE / ABORT; empty alongside an empty sequence
uint32 token      # echoed from SequenceRequest; 0 when nothing is running
