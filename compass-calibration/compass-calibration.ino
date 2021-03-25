/*
QMC5883LCompass.h Library Calibration
*/
#include <QMC5883LCompass.h>

QMC5883LCompass compass;

int calibrationData[3][2];
bool changed = false;
bool done = false;
int t = 0;
int c = 0;

void setup()
{
    Serial.begin(9600);
    compass.init();

    Serial.println("This will provide calibration settings for your QMC5883L chip. When prompted, move the magnetometer in all directions until the calibration is complete.");
    Serial.println("Calibration will begin in 5 seconds.");
    delay(5000);
}

void loop()
{
    int x, y, z;

    // Read compass values
    compass.read();

    // Return XYZ readings
    x = compass.getX();
    y = compass.getY();
    z = compass.getZ();

    changed = false;

    if (x < calibrationData[0][0])
    {
        calibrationData[0][0] = x;
        changed = true;
    }
    if (x > calibrationData[0][1])
    {
        calibrationData[0][1] = x;
        changed = true;
    }

    if (y < calibrationData[1][0])
    {
        calibrationData[1][0] = y;
        changed = true;
    }
    if (y > calibrationData[1][1])
    {
        calibrationData[1][1] = y;
        changed = true;
    }

    if (z < calibrationData[2][0])
    {
        calibrationData[2][0] = z;
        changed = true;
    }
    if (z > calibrationData[2][1])
    {
        calibrationData[2][1] = z;
        changed = true;
    }

    if (changed && !done)
    {
        Serial.println("CALIBRATING... Keep moving your sensor around.");
        c = millis();
    }
    t = millis();

    if ((t - c > 5000) && !done)
    {
        done = true;
        Serial.println("DONE. Copy the line below and paste it into your projects sketch.);");
        Serial.println();

        Serial.print("#define COMPAS_CALIBRATION_X_MIN ");
        Serial.println(calibrationData[0][0]);
        Serial.print("#define COMPAS_CALIBRATION_X_MAX ");
        Serial.println(calibrationData[0][1]);
        Serial.print("#define COMPAS_CALIBRATION_Y_MIN ");
        Serial.println(calibrationData[1][0]);
        Serial.print("#define COMPAS_CALIBRATION_Y_MAX ");
        Serial.println(calibrationData[1][1]);
        Serial.print("#define COMPAS_CALIBRATION_Z_MIN ");
        Serial.println(calibrationData[2][0]);
        Serial.print("#define COMPAS_CALIBRATION_Z_MAX ");
        Serial.println(calibrationData[2][1]);
    }
}