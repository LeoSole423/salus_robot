# Atomic, latched declaration of the effective system profile and its
# capabilities. Dynamic sensor health may update individual states, but a
# failure must never silently select a less-protected profile.

std_msgs/Header header
string profile
CapabilityState[] capabilities
