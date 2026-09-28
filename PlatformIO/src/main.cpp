#include <Arduino.h>
#include <BluetoothDebugWorkflow.h>
#include <config.h>

BluetoothDebugWorkflow debugGui(
    BluetoothConfig::PORT,
    HerkulexConfig::PORT
);

void setup()
{
    debugGui.begin();
}

void loop()
{
    debugGui.update();
    debugGui.updateLocalCompetition();
}
