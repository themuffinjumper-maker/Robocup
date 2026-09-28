# HX12K wiring through the Digital Level Shift board

This setup uses one servo on Digital Raw 1 channel D30.

## CPU to level shifter

Connect the CPU board's 8-pin **DIGITAL RAW1** header to the Digital Level
Shift board's 8-pin input with the keyed straight-through cable. The CPU header
is labelled, in order:

`3V | GND | D30 | D31 | D32 | D33 | GND | 3V`

The four output mappings are:

- Output A -> D33
- Output B -> D32
- Output C -> D31
- Output D -> D30

D30-to-D was confirmed by testing the physical boards. The other three follow
the same reversed connector order.

## External servo power

1. With power disconnected, place the level-shifter power jumper in **Ex**.
2. Connect the external servo supply to the level shifter's external-power
   input, observing positive and negative polarity.
3. Use a supply voltage suitable for the HX12K and capable of its peak/stall
   current. Do not power the servo from a Teensy 3.3 V pin.
4. The level-shifter board establishes the required shared ground through its
   CPU cable; do not bypass the board with the servo power lead.

## Servo to output D

Plug the HX12K into output **D** on the right side of the level-shifter board.
Match the connector markings rather than relying only on wire colours:

- Servo ground (normally brown or black) to GND.
- Servo power (normally red) to the shifted/external supply rail.
- Servo signal (normally orange, yellow, or white) to signal.

Never reverse the power and ground pins. Check the board silkscreen and the
servo lead before applying external power.

## Firmware behaviour

Each `Hx12kServo` uses a 1000-2000 microsecond pulse range and maps 0-135 degrees
across that range. All four start disabled. In the debug GUI, enter Debug Mode,
press Run Robot, then use **Set HX12K servo angles**. Tick only the outputs you
want to move and set an independent angle for each. Begin near the centre (67.5
degrees) with linkages disconnected. **Disable HX12K output**, global STOP, and
leaving Debug Mode detach all four pulse outputs.

Output **B** (Teensy D32 via the level shifter) is now the storage-release
gate: 50 degrees closed, 10 degrees open. The debug GUI has paired OPEN GATE
and CLOSE GATE buttons in the HX12K command card and Arena View. Check the
physical direction with the linkage clear before trying an autonomous unload.
