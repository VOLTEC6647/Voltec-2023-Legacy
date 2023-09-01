/**
 * Written by Juan Pablo Gutiérrez
 * Hanldes all subsystem state telemetry
 */

package com.team6647.util.shuffleboard.states;

import com.team6647.subsystems.CubeintakeSubsystem;
import com.team6647.util.Constants.ShuffleboardConstants;

public class SubsystemStatesTelemetry {
    CubeintakeSubsystem cubeintakeSubsystem;

    public SubsystemStatesTelemetry(CubeintakeSubsystem cubeintakeSubsystem){
        this.cubeintakeSubsystem = cubeintakeSubsystem;
        ShuffleboardConstants.kShuffleboardTab.add("Cube Intake Pivot Chooser", cubeintakeSubsystem.getPivotChooser());
        ShuffleboardConstants.kShuffleboardTab.add("Cube Intake Roller Chooser", cubeintakeSubsystem.getRollerChooser());

    }
    
}
