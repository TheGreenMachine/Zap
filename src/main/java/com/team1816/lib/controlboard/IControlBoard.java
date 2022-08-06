package com.team1816.lib.controlboard;

public interface IControlBoard {
    boolean getAsBool(String getName);
    double getAsDouble(String getName);

    boolean getAsBoolJoystickPositive(String getName);
    boolean getAsBoolJoystickNegative(String getName);

    boolean update();

    void outputToSmartDashboard();
}
