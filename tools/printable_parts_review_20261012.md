# Printable-part review — 12 October 2026

The STL files are geometrically valid in the checks performed. They are not
all physically verified designs. No printable geometry was edited for this
review. The inspection covered the hardware documentation, part manifests,
CAD generators, assembly and fit tests, simulator consumers, launcher state
machines, and all 145 committed hardware STL meshes.

## File validity and purpose

All 145 meshes have finite coordinates, closed surfaces, consistent winding
and positive net volume. No zero-area faces were found at a 1e-12 mm²
area threshold. There are 129 current manifest parts and 16 additional
SO101 upstream reference meshes. The SO101 moving jaw has two connected shells
in both its original and printable mesh. The smaller shell has negative signed
volume and lies entirely inside the outer shell: it describes an internal
void, not a loose component or a new conversion defect.

The files include vehicle structures and mounts, sensor cradles, servo-driven
joints and launcher parts, plus SO101 arm components. Their transforms and
assembly quantities matter: individual print orientations are not simulation
assembly poses. The aerial simulator now consumes the assembled STL geometry
and a shared mass/inertia manifest instead of estimating the airframe from
primitive shapes. Its loaded mass is **3.164661 kg**, an estimate that still
needs a scale measurement of the assembled aircraft.

The full CAD/metadata/documentation suite previously passed 258 tests. Its
freshness comparison now checks surfaces at the declared 0.05 mm mesh tolerance,
so matching bounds, volume and triangle counts can no longer hide a changed
surface. Read-only checking, partial builds, export paths and cleanup were
also repaired. The later all-file mesh scan reports no failed meshes.

## Remaining fit evidence

`verified` in the manifests describes the dimensional source, not a successful
print-and-assembly trial. These counts identify interfaces that especially need
measurement with the actual purchased components:

| Platform | Manifest parts | Fit-test parts | Unverified mating interfaces |
|---|---:|---:|---:|
| Aerodrone | 21 | 10 | 10 |
| Airplane | 11 | 9 | 14 |
| Boat | 2 | 1 | 1 |
| Lunar rover | 17 | 9 | 9 |
| Racecar turret adapter | 3 | 0 | 0 |
| Racecarneo | 17 | 8 | 8 |
| SO101 | 27 | 4 | 2 |
| Submarine | 6 | 2 | 3 |
| Standalone turret | 25 | 13 | 12 |

Priorities are actual servo horn and connector clearances; motor screw depth;
camera and radio connector access; dart dimensions and sliding fits; and
printed fastener/insert fits. Use the existing fit-test parts before full
assemblies and record the printer, material, measured dimensions and result.
Mesh closure cannot establish layer strength, creep, fatigue, vibration or
watertightness of an assembled hull. Those remain physical test requirements.

## Launcher mechanics

The standalone and car turrets use the same spring/plunger, sliding breech,
clip, follower and sear design. The drone adapts the mounting and orientation
and adds a dart-retention detent. The clip holds up to six Elite-size foam
darts; it is an integrated screw-fastened clip, **not a removable standard
Nerf magazine/cartridge**. The loaded aircraft budget represents five darts
in the clip and one in the chamber.

The controller primes, catches, chambers and triggers on each requested cycle.
Chambering belongs to the next firing cycle; it does not automatically chamber
another dart immediately after each launch. Simulated cycles verify the
command sequence, servo travel, sear thresholds, aborts and target advancement.
They do not model foam compression, friction, spring variation, feed jams,
sealing, real servo load or successful dart passage. There is no physical
feed/exit confirmation, so a software "fired" result is not proof that a dart
left the barrel.

The supported conclusion is that the nominal CAD and controller sequence are
consistent, while reliable physical feeding and repeated launching remain
unverified. An assembled mechanism is required to resolve that. Useful next
improvements are a documented repeat-cycle and orientation test, measured
assembly clearances, jam/empty feedback, and a physical dart-presence/exit
measurement. No geometry, spring or physical automatic firing was changed.

Machine-readable scan: [stl_mesh_review.json](validation/20261012/stl_mesh_review.json).
