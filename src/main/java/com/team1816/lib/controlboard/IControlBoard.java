package com.team1816.lib.controlboard;

/**
 * A basic interface for accessing controller outputs
 * @see com.team1816.season.controlboard.ControlBoard
 */
public interface IControlBoard {
    boolean getAsBool(String getName);
    double getAsDouble(String getName);
}
