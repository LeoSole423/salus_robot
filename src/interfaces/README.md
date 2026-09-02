# interfaces

This is a deliberately temporary **ROS wire-compatibility** package. Its ROS
package name is exactly `interfaces` so that the read-only coexistence adapters
can subscribe to the types published by the deployed `ROS2_SALUS` system.

It contains only `CmdVelFinal` and `DriveTelemetry`, copied byte-for-byte at the
message-definition level from
`ROS2_SALUS@f35834989b041f51dd325c626d2338e2232d9e53`. Although equivalent
canonical contracts exist in `salus_interfaces`, ROS 2 type identity includes
the package name, so `interfaces/msg/X` cannot connect to
`salus_interfaces/msg/X`.

This is not a canonical API and new SALUS code must not use it. It exists only
while `ROS2_SALUS` participates in coexistence and must be removed at its
controlled retirement. Adding another legacy type requires explicit hardware
evidence and an architecture decision; copying the legacy package wholesale is
forbidden.
