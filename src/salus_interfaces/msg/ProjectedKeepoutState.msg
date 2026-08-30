# Atomic, revisioned active keepout geometry in map coordinates.
# Published only after the corresponding zone document becomes authoritative.
# revision increases for each accepted replacement during one publisher lifetime.
# Consumers must treat any revision change as replacement; a manager restart may reset it.

std_msgs/Header header
uint64 revision
ProjectedKeepoutPolygon[] polygons
