/**
 * Written by Juan Pablo Gutiérrez
 * Contains all logged robot telemetry
 */

package com.team6647.util.shuffleboard.tabs;

import com.andromedalib.shuffleboard.ShuffleboardTabBase;
import com.team6647.subsystems.CubeintakeSubsystem;
import com.team6647.util.shuffleboard.states.SubsystemStatesTelemetry;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class DebugTab extends ShuffleboardTabBase {

    private CubeintakeSubsystem cubeintakeSubsystem = CubeintakeSubsystem.getInstance();

    private GenericEntry cubeIntakePivotPosition;
    private GenericEntry cubeIntakePivotState;
    private GenericEntry cubeRollerState;

    public DebugTab(ShuffleboardTab tab) {
        new SubsystemStatesTelemetry(cubeintakeSubsystem);

        cubeIntakePivotPosition = tab.add("Cube Intake Position", cubeintakeSubsystem.getPivotPosition())
                .withWidget(BuiltInWidgets.kAccelerometer).getEntry();
        cubeIntakePivotState = tab.add("Cube Intake Pivot State", cubeintakeSubsystem.getPivotState()).getEntry();
        cubeRollerState = tab.add("Cube Roller State", cubeintakeSubsystem.getRollerState()).getEntry();
    }

    @Override
    public void updateTelemetry() {
        cubeIntakePivotPosition.setDouble(cubeintakeSubsystem.getPivotPosition());
        cubeIntakePivotState.setString(cubeintakeSubsystem.getPivotState().toString());
        cubeRollerState.setString(cubeintakeSubsystem.getRollerState().toString());
    }

}
