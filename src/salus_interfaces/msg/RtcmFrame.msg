# Validated RTCM transport frame. The payload is intentionally carried only on
# this transport contract; status and diagnostic contracts must not echo it.

std_msgs/Header header

# Stable logical source identifier, never a credential or filesystem path.
string source_id

# Monotonic sequence local to source_id. Consumers must expose discontinuities
# rather than inferring global ordering across sources.
uint64 sequence

# Non-empty, bounded, and CRC-validated payload enforcement belongs to the
# producer policy before publication.
uint8[] data
