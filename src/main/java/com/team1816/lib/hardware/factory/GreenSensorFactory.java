package com.team1816.lib.hardware.factory;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.I2C;

public class GreenSensorFactory {

    private ColorSensorV3 createColorSensor(I2C.Port id) {
        var sensor = new ColorSensorV3(id);
        configureColorSensorV3(sensor);
        return sensor;
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

    private DigitalInput createBeamBreakSensor(int id) {
        return new DigitalInput(id);
    }


}
