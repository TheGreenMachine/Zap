package com.team1816.lib.hardware.factory;

import com.revrobotics.ColorSensorV3;
import com.team1816.lib.hardware.components.sensors.GhostColorSensor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;

public class GreenSensorFactory {

    public ColorSensorV3 createColorSensor(I2C.Port port, boolean ghost) { // Robot factory will tell it to ghost so if port = -1, make it 0
        if (ghost) {
            var sensor = new GhostColorSensor(port);
            return sensor;
        } else {
            var sensor = new ColorSensorV3(port);
            configureColorSensorV3(sensor);
            return sensor;
        }
    }

    private void configureColorSensorV3(ColorSensorV3 sensor) {
        configureColorSensorV3(
            sensor,
            ColorSensorV3.ColorSensorResolution.kColorSensorRes16bit,
            ColorSensorV3.ColorSensorMeasurementRate.kColorRate25ms,
            ColorSensorV3.GainFactor.kGain3x
        );
    }

    private void configureColorSensorV3(
        ColorSensorV3 sensor,
        ColorSensorV3.ColorSensorResolution res,
        ColorSensorV3.ColorSensorMeasurementRate mr,
        ColorSensorV3.GainFactor gf
    ) {
        sensor.configureColorSensor(res, mr, gf);
    }

    public DigitalInput createBeamBreakSensor(int id) {
        return new DigitalInput(id);
    }



}
