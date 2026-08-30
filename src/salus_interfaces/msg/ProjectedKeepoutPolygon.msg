# One accepted keepout polygon projected into the map frame.
# MultiPolygon features are represented as multiple entries sharing zone_id.

string zone_id
geometry_msgs/Polygon outer
geometry_msgs/Polygon[] holes
