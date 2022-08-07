package com.team1816;

import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.team1816.lib.Injector;
import com.team1816.lib.hardware.SubsystemConfig;
import com.team1816.lib.hardware.components.IPigeonIMU;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.pcm.ICompressor;
import com.team1816.lib.hardware.factory.RobotFactory;

public class TestUtil {

    public static void SetupMockRobotFactory(RobotFactory mockFactory) {
        if (mockFactory == null) mockFactory = mock(RobotFactory.class);
        // any generic calls from Constants typically
        when(mockFactory.getConstant(anyString(), anyDouble()))
            .thenAnswer(input -> input.getArguments()[1]);
        //for infrastructure
        when(mockFactory.getCompressor()).thenReturn(mock(ICompressor.class));
        when(mockFactory.getPigeon()).thenReturn(mock(IPigeonIMU.class));
        // for motors
        when(mockFactory.getMotor(anyString(), anyString()))
            .thenReturn(mock(IGreenMotor.class));
        when(mockFactory.getMotor(anyString(), anyString(), any(IGreenMotor.class)))
            .thenReturn(mock(IGreenMotor.class));
        // for subsystems
        var config = new SubsystemConfig();
        config.implemented = false;
        when(mockFactory.getSubsystem(anyString())).thenReturn(config);
        Injector.register(mockFactory);
    }
}
