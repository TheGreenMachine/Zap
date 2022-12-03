package com.team1816.lib.hardware.components.sensor;

import edu.wpi.first.wpilibj.util.Color;

public interface IColorSensor {
    int getId();

    String getName();

    Color getColor();

    int getRed();

    int getGreen();

    int getBlue();
}
