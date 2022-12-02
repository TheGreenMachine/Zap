package com.team1816.lib.hardware.factory;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

public class GreenSensorFactory {

    private DigitalInput colorSensorInput;
    private DigitalInput beamBreakSensorInput;
    private DigitalOutput colorSensorOutput;
    private DigitalOutput beamBreakSensorOutput;

    private String sensorName;

    private String NAME;

    private int CAN_ID;

    public GreenSensorFactory(String n, String na, int CI){
        sensorName = n;
        NAME = na;
        CAN_ID = CI;

        if (n == "color"){
            colorSensorInput = new DigitalInput(0);
            colorSensorOutput = new DigitalOutput(0);
        }

        else if (n == "beam break"){
            beamBreakSensorInput = new DigitalInput(0);
            beamBreakSensorOutput = new DigitalOutput(0);
        }

        else {
            colorSensorInput = null;
            colorSensorOutput = null;
            beamBreakSensorInput = null;
            beamBreakSensorOutput = null;
        }
    }

    private DigitalInput getColorSensorInput(){
        return colorSensorInput;
    }

    private DigitalInput getBeamBreakSensorInput(){
        return beamBreakSensorInput;
    }

    private DigitalOutput getColorSensorOutput(){
        return colorSensorOutput;
    }

    private DigitalOutput getBeamBreakSensorOutput(){
        return beamBreakSensorOutput;
    }




}
