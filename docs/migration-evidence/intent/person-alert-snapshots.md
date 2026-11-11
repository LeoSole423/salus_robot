# Person alert snapshots — 2026-11-11

Operator needs an image with boxes to verify person alerts. Use exactly the
already sampled frame that generated each confirmed alert; no additional video
reader or inference. Annotate normalized boxes/confidence and encode JPEG only
on alerts. Keep at most 50 images, each at most 256 KiB, in memory alongside the
existing volatile alert history. Eviction and restart remove images. Historical
alerts cannot acquire evidence retroactively. Failure to encode must preserve
the alert and explicitly show no capture. Serve read-only images through the
existing private listener/origin policy, never arbitrary filesystem paths.
Cockpit loads an image only when the operator opens that alert. Navigation and
camera configurations remain unchanged; only the detector needs a restart.
