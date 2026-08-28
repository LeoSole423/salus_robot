# Canonical RTK/GNSS observation status. GNSS fix quality is derived only from
# receiver telemetry; receiving recent RTCM corrections never determines it.
# status_detail and source_id must not contain credentials, configuration paths,
# or RTCM payloads.

std_msgs/Header header

uint8 UNKNOWN=0
uint8 NO_FIX=1
uint8 AUTONOMOUS=2
uint8 DGPS=3
uint8 RTK_FLOAT=4
uint8 RTK_FIXED=5

uint8 ACQUISITION_DISABLED=0
uint8 ACQUISITION_DISCONNECTED=1
uint8 ACQUISITION_CONNECTED_NO_DATA=2
uint8 ACQUISITION_RECEIVING=3
uint8 ACQUISITION_STALE=4
uint8 ACQUISITION_ERROR=5

uint8 BACKEND_DISABLED=0
uint8 BACKEND_PIXHAWK_MAVROS=1
uint8 BACKEND_DIRECT_USB=2

uint8 DELIVERY_DISABLED=0
uint8 DELIVERY_IDLE=1
uint8 DELIVERY_DELIVERING=2
uint8 DELIVERY_STALE=3
uint8 DELIVERY_ERROR=4

# GNSS solution quality reported by receiver telemetry, independent of RTCM
# acquisition, freshness, and delivery state.
uint8 fix_quality

# State of correction acquisition from the selected source.
uint8 acquisition_state

# Explicitly selected correction-delivery backend; it never falls back
# automatically after a fault.
uint8 delivery_backend

# Observed state of delivery to the selected backend.
uint8 delivery_state

# Raw receiver-specific fix type when available; -1 means unavailable.
int8 receiver_fix_type

# Satellites visible to the receiver; 255 means unknown.
uint8 satellites_visible

# Whether corrections are currently within the configured freshness bound.
bool corrections_fresh
float32 correction_age_s
uint64 received_count
uint64 crc_error_count

# Stable logical correction-source identifier, never a credential or path.
string source_id

# Sanitized, bounded operator-facing detail with no secrets or payload data.
string status_detail
