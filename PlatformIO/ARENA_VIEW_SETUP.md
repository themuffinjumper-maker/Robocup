# Arena View setup and first calibration

The Debug GUI now has an **Arena View** tab. The Teensy firmware reads a BNO055
on raw I2C1 (same bus as the SEN0628 8x8 TOF), checking addresses 0x28 then
0x29. It checks I2C0 as a fallback. Keep the IMU and 8x8 device on their
provided four-wire I2C cables and do not change the motor wiring. The new IMU
telemetry fields are `imu.available`, `imu.valid`, `imu.bus`, `imu.address`,
`imu.heading_deg`, `imu.roll_deg`, `imu.pitch_deg`, quaternion, linear
acceleration, gravity, temperature, operation mode, self-test, fusion status,
system status/error, and four calibration levels. `imu.valid` means the I2C
register reads succeeded. `imu.fusion_running` means the BNO055 reports system
status 5 (fusion algorithm running). A raw `imu.system_error` value is only an
active error when `imu.system_error_active` is true; the chip may retain a raw
code while operating normally.

The 2D outline uses the supplied 2400 × 4900 mm arena dimensions. The walls
are 400 mm high; their height cannot be shown in this overhead view. The robot
starts at the centre of a *local* coordinate frame. It is not localised to an
absolute arena starting position. The view draws the robot, encoder trail,
current TOF rays and a square evidence grid. Grey cells are unobserved, green
cells have been crossed by a valid range ray, and red cells are valid measured
endpoints. Repeated readings strengthen or weaken the evidence; a colour is
not proof that the cell is safe or blocked. The square size defaults to 10 mm
and can be changed in the tab. It uses the centre two
rows of the 8x8 matrix as a horizontal slice; the other rows are not projected
onto the floor. Black/angled surfaces may give erroneous or absent ranges.

Powered calibration now gives encoder 1 **0.09094 mm/count** and encoder 2
**0.09592 mm/count**. Encoder 1 is inverted and encoder 2 is not, so positive
distance means robot-forward for both. These values come from the first two
consistent straight trials: 775 mm over 8522 encoder-1 counts and 8080
encoder-2 counts. A third claimed 340 mm trial produced almost the same counts
as the 385–390 mm trials and was rejected as an approximately 12% outlier.
Both scales remain editable in Arena View. Reset the map after changing any
geometry or encoder calibration. The wheel-track setting defaults to the supplied 40 mm centre-to-
centre measurement, but it should still be measured because it is unusually
small for 80 mm wheels. When the BNO055 fusion engine is running, its heading
supplies the turn angle even if the separate system calibration field remains
zero. Encoder-only turns require a correct wheel track whenever IMU fusion is
unavailable.

The BNO055 is used for orientation, not standalone translational displacement.
Double-integrating its linear acceleration would rapidly accumulate bias and
vibration error. The map therefore uses encoder distance for translation and
the fused BNO055 heading for direction. Arena View shows local X/Y pose, total
wheel travel and current encoder-derived linear speed. This is dead reckoning:
wheel slip and scale error still accumulate, and the TOFs do not yet perform
absolute pose correction against the arena walls.

The mapper suppresses BNO055 yaw changes while the combined encoder movement
is below four counts. This stationary lock prevents IMU drift from rotating
successive range observations around a robot which has not moved; the IMU
baseline is still refreshed so the accumulated drift is not applied on the
next movement. Each point sensor and each projected 8x8 column also has an
independent short median filter. A range jump greater than 150 mm or 35% must
be repeated consistently before it is added to the persistent map. Raw sensor
telemetry and the separate 8x8 view remain unfiltered. Point-sensor zero and
8191 sentinel returns, along with any other values outside the configured
10–3500 mm mapping range, are rejected before this temporal filter. The 10 mm
lower bound keeps a zero sentinel invalid while retaining close measurements.

The current Wiring Guide names six point sensors: XSHUT1 and XSHUT2 are long
range; XSHUT5, XSHUT0, XSHUT3, and XSHUT4 are short range. These same six ports are
now in `debug_config.json` and their individual telemetry feeds the grid.
The GUI joins them to Wiring Guide entries by XSHUT port. Each has editable
robot-right offset, forward offset, and pointing angle. The starting offsets
are rough layout guesses based on their labels; all pointing angles start at
zero until physically measured. Wiring Guide edits update the displayed
labels; port additions also require an entry in `debug_config.json`, rebuild,
and upload before the Teensy can stream that device. The Arena View status
flags a guide port with no matching firmware configuration.

The 8x8 horizontal field of view defaults to a 60° **estimate**, also editable.
The 8x8 display and Arena View start horizontally flipped so image-left means
robot-left when viewed from the robot, not from in front of it. The toggle on
the 8x8 tab changes both displays together and persists locally. Sensor wiring
names are based on physical XSHUT assignments, not the visual flip.

Arena View includes an always-visible raw TOF panel beneath the map. It shows
each point sensor by Wiring Guide label/port and the complete raw X0–X7,
Y0–Y7 matrix with frame, bus and address. Values marked invalid are displayed
but never mapped. These readouts precede the temporal mapping filters.

All 64 matrix zones now contribute to the floor plan instead of using only the
two centre rows. A top-down map cannot represent the matrix's vertical field
of view as extra horizontal bearings. The mapper therefore takes the median
of the valid Y0–Y7 ranges in each real X column, requires at least three valid
pixels, waits for two consistent frames, and paints that column's complete
horizontal angular sector as a filled wedge. The eight wedges form the cone.
The raw table remains the literal source for all individual zone values.

Matrix ranges are projected as forward depth relative to the module centreline,
not as equal-radius points on a circular arc. Each horizontal ray is extended by
`depth / cos(ray angle)`, so a flat plate square to the sensor produces a straight
red boundary across the cone. Natural per-column measurement differences can
still make the boundary slightly stepped because the map uses 10 mm cells.

The map now defaults to **10 mm (1 cm) cells**. The map is zoomable with the
mouse wheel and pannable by dragging; use Reset map zoom / pan to return to the
full-arena view. The GUI caches the grid as an image so the finer resolution
does not require drawing more than 100,000 individual rectangles each frame.
If an older installation was still using the former 100 mm default, the app
migrates it to 10 mm once; a deliberately customised size is retained.

The robot diagram in Arena View is a mounting-layout editor, not a measured
chassis drawing. It shows six point sensors and the 8x8 module. Drag a sensor
marker to set its robot-right and forward offsets; drag its white arrow tip
to set beam direction. Every sensor also has numeric offset/angle controls
and a top/bottom height-layer selector. Cyan circles mean top and orange
squares mean bottom. Settings persist locally. The default offsets are layout
guesses; importantly, top/bottom means vertical mounting height, not forward/
rearward position. Confirm actual mount locations and beam angles by hand.

Purple cells indicate a **possible weight**, not a confirmed object class.
The same observation must recur twice: a bottom point sensor sees a nearer
return while a nearby, similarly aimed top point sensor sees at least 150 mm
farther. When there is no matching top point sensor, the recent 8x8 centre-row
slice may provide the top comparison if its layer is set to top. The distance
gap is adjustable in Arena View (150 mm by default). Missing,
timed-out, old, or unaligned upper measurements never trigger purple. At a
black angled wall, upper and lower sensors can disagree for other reasons;
physically test the rule before using it to drive collection behaviour.

After uploading the rebuilt firmware, connect the GUI, check `imu.available`
and `imu.bus == 1`, and turn the robot gently by hand. The blue triangle
should rotate in the same sense. Move it forward a measured distance and
verify the blue trail direction and scale. Then check a single close obstacle
in front and on each side. Do not use this display for autonomous movement or
collision avoidance until these checks and a wall/encoder drift test pass.
