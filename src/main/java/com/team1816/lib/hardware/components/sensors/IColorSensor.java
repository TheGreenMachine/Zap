package com.team1816.lib.hardware.components.sensors;

import edu.wpi.first.wpilibj.util.Color;

public interface IColorSensor {
    int getId();

    Color getColor();

    int getRed();

    int getGreen();

    int getBlue();

    int[] getRGB();
}
