# Bluetooth debug setup

## Hardware

The supplied CH9143 boards are a pre-matched transparent serial bridge. Plug
the robot-side board into the carrier board's `SERIAL1` connector.

| CH9143 | Teensy 4.0 |
|---|---|
| TX | pin 0 / RX1 |
| RX | pin 1 / TX1 |
| GND | GND |
| VCC | supply required by the exact CH9143 board |

TX and RX cross over. Confirm the CH9143 board's supply and UART logic voltage
from its markings/documentation before applying power. Teensy 4.0 GPIO is 3.3 V
and is not 5 V tolerant.

Both CH9143 ends and the firmware use 115200 baud, 8 data bits, no parity, and
1 stop bit. The PC-side unit appears in Windows as a COM port.

## Firmware

1. Open the `PlatformIO` folder in VS Code with PlatformIO installed.
2. Build the `teensy40` environment. ArduinoJson is installed automatically.
3. Connect the Teensy by USB and upload.
4. Firmware starts with all Herkulex torque disabled.

## GUI

```powershell
cd tools\BluetoothDebugGUI
.\setup.ps1
.\run_gui.ps1
```

Select the port labelled `USB-BLE-SERIAL CH9143`, choose 115200 baud, and click **Connect**. The GUI
sends `hello` and `request_definitions`; Teensy replies with state, parameters,
commands, and periodic telemetry.

To test the link without actuating anything:

```powershell
.\.venv\Scripts\python.exe .\smoke_test.py COM10
```

Replace `COM10` if Windows assigns a different CH9143 port.

Bluetooth uses `SERIAL1`. Plug the Herkulex bus into the separate `SERIAL2`
connector; a single UART cannot carry both protocols.

## First bench test

1. Test with the robot lifted and mechanisms clear.
2. Connect and confirm Frames/s and Signals/s are non-zero.
3. Confirm the Parameters and Commands tabs populate.
4. Press **Ping Teensy** and confirm `Pong from Teensy` appears in Logs.
5. Enter Debug Mode and use **Set Herkulex angle** with a small safe movement.
6. Press STOP and confirm servo torque is disabled.

The GUI stop command is not an emergency stop. Keep a physical power cutoff
available during actuator testing.
