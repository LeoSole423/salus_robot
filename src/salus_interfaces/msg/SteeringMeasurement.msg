salus_interfaces/MeasurementMetadata metadata

uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_STEERING_MOTOR_SHAFT=1
uint8 SOURCE_STEERING_LINKAGE=2
uint8 SOURCE_ROAD_WHEEL=3
uint8 SOURCE_VIRTUAL_CENTER_WHEEL=4
uint8 source_type

uint16 FIELD_POSITION=1
uint16 FIELD_ANGULAR_VELOCITY=2
uint16 available_fields

# Signed physical quantities in REP-103 units. Positive position turns the
# vehicle left after applying the configured mechanical sign. Road-wheel and
# virtual-center angles must not be reported as measured linkage angles.
float64 position_rad
float64 angular_velocity_rad_s
