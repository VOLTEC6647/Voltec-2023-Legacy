/**
 * Written by Juan Pablo Gutiérrez
 * Contains all logged robot telemetry
 */

package com.team6647.util.shuffleboard.tabs;

import com.andromedalib.shuffleboard.ShuffleboardTabBase;
import com.team6647.subsystems.IntakeSubsystem;
import com.team6647.subsystems.PivotCubeSubsystem;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class DebugTab extends ShuffleboardTabBase {

    private PivotCubeSubsystem cubeintakeSubsystem = PivotCubeSubsystem.getInstance();
    private IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    private GenericEntry cubeIntakePivotPosition;
    private GenericEntry cubeIntakePivotState;
    private GenericEntry cubeRollerState;

    public DebugTab(ShuffleboardTab tab) {

        cubeIntakePivotPosition = tab.add("Cube Intake Position", cubeintakeSubsystem.getPivotPosition())
                .withWidget(BuiltInWidgets.kAccelerometer).getEntry();
        cubeIntakePivotState = tab.add("Cube Pivot State", cubeintakeSubsystem.getPivotState().toString()).getEntry();
        cubeRollerState = tab.add("Cube Roller State", intakeSubsystem.getRollerState().toString()).getEntry();
    }

    @Override
    public void updateTelemetry() {
        cubeIntakePivotPosition.setDouble(cubeintakeSubsystem.getPivotPosition());
        cubeIntakePivotState.setString(cubeintakeSubsystem.getPivotState().toString());
        cubeRollerState.setString(intakeSubsystem.getRollerState().toString());
    }

}
