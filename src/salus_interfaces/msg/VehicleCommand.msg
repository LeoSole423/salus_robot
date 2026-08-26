# Atomic, backend-independent command at the vehicle kinematic boundary.
# AckermannDrive steering_angle is the virtual center-wheel angle in radians;
# speed is signed m/s. Zero acceleration, jerk or steering-angle velocity means
# no rate constraint, following ackermann_msgs semantics.

uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_AUTO=1
uint8 SOURCE_MANUAL=2
uint8 SOURCE_SAFETY=3

float32 BRAKE_RELEASED=0.0
float32 BRAKE_FULL=1.0

std_msgs/Header header
builtin_interfaces/Duration valid_for
uint8 source
bool drive_enabled
bool emergency_stop
float32 brake_ratio
ackermann_msgs/AckermannDrive drive
