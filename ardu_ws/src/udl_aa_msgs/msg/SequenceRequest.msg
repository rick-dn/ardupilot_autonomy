# Commander -> sequence node. One sequence to run.
#
# A layer above VehicleCommand: that carries a single primitive (ARM, TAKEOFF,
# GOTO_LOCAL) on the sequences -> flight stack hop, while this names a whole
# multi-step sequence on the commander -> sequences hop. The commander decides
# which sequence should run; it never issues a primitive itself.
#
# The sequence is named, not numbered, and the token deliberately does not carry
# the name. The flight stack packs a sequence id into the high bits of its token
# only because VehicleCommand has no identity field to use; here there is one,
# so packing would put identity in two places that can disagree. A name also
# survives `ros2 topic echo` legibly, and means adding a sequence is one row in
# one registry rather than a name-to-id table both nodes must agree on.
#
# Published on change only. A standing safety condition asks for its sequence
# every tick, and republishing at tick rate would make the sequence node
# responsible for deduplicating. The exception is `manual`, whose arguments are
# the point - each publish there is a different request, not a repeat.
#
# `abort` is a sequence name like any other; only its priority is special.

std_msgs/Header header

string    sequence       # registry name: takeoff, smart_rtl, abort, manual
uint32    token          # request id, monotonic; echoed back in SequenceStatus

string[]  param_names    # zipped into the dict on_start(params) expects;
float64[] param_values   # manual carries its axis here. Empty for most.
