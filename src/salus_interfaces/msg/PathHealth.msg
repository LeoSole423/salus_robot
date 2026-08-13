# Health of the active global path. Produced by salus_navigation/path_health.
uint8 KEEP_PATH=0
uint8 REPLAN=1
uint8 STOP_AND_WAIT=2

builtin_interfaces/Time stamp
uint8 state
string reason
float64 costmap_age_s
int16 max_cost
uint32 checked_samples
float64 cross_track_error_m
