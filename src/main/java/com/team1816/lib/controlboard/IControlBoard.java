package com.team1816.lib.controlboard;

public interface IControlBoard {
    boolean getAsBool(String getName);
    double getAsDouble(String getName);

    boolean update();

    void outputToSmartDashboard();
}
