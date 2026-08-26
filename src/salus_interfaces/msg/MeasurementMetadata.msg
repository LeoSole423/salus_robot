std_msgs/Header header

# Stable logical identifier of the physical source, for example
# "rear_traction_motor" or "front_steering_linkage". It is configuration,
# not a device path or transport endpoint.
string source_id

uint8 STATUS_UNKNOWN=0
uint8 STATUS_OK=1
uint8 STATUS_STALE=2
uint8 STATUS_INVALID=3
uint8 STATUS_UNAVAILABLE=4
uint8 status

# Sequence local to the adapter. A discontinuity exposes a source reset; it
# does not impose ordering between different sources.
uint32 sequence
