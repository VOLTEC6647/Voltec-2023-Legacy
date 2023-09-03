/**
 * Written by Juan Pablo Gutiérrez 
 * 02 09 2023
 */
package com.team6647.commands.hybrid.Intake;

import com.team6647.subsystems.PivotCubeSubsystem;
import com.team6647.subsystems.PivotCubeSubsystem.PivotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ToggleIntake extends InstantCommand {
  private PivotCubeSubsystem cubeintakeSubsystem;

  /** Creates a new ToggleIntake. */
  public ToggleIntake(PivotCubeSubsystem cubeintakeSubsystem) {
    this.cubeintakeSubsystem = cubeintakeSubsystem;

    addRequirements(cubeintakeSubsystem);
  }

  @Override
  public void initialize() {
    cubeintakeSubsystem.resetPID();
    PivotState newState = (cubeintakeSubsystem.getPivotState() == PivotState.HOMED) ? PivotState.EXTENDED : PivotState.HOMED;
    cubeintakeSubsystem.changePivotState(newState);
  }
}
