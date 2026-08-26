salus_interfaces/MeasurementMetadata metadata

uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_MOTOR_SHAFT=1
uint8 SOURCE_DRIVE_WHEEL=2
uint8 SOURCE_VEHICLE_GROUND=3
uint8 source_type

uint16 FIELD_POSITION=1
uint16 FIELD_ANGULAR_VELOCITY=2
uint16 FIELD_LINEAR_VELOCITY=4
uint16 available_fields

# These masks are disjoint and their union must equal available_fields. They
# allow one sample to carry, for example, measured position and calculated
# velocity without claiming both were measured.
uint16 measured_fields
uint16 calculated_fields
uint16 inferred_fields

# Signed physical quantities in REP-103 units. Positive velocity means forward
# vehicle motion after applying the configured mechanical sign. A field is
# meaningful only when its FIELD_* bit is present and metadata.status is OK.
float64 position_rad
float64 angular_velocity_rad_s
float64 linear_velocity_mps
