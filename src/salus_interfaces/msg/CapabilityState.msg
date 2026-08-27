# Declared capability state for the active system profile. This describes
# configuration and observed availability without inferring health from a
# missing topic. ENABLED_BY_PROFILE is configuration, not proof of live health.

uint8 STATE_UNKNOWN=0
uint8 STATE_NOT_INSTALLED=1
uint8 STATE_DISABLED_BY_PROFILE=2
uint8 STATE_UNAVAILABLE=3
uint8 STATE_INVALID=4
uint8 STATE_STALE=5
uint8 STATE_FAILED=6
uint8 STATE_ENABLED_BY_PROFILE=7
uint8 STATE_READY=8

string capability_id
uint8 state
bool required
bool enabled
string[] source_ids
string detail
