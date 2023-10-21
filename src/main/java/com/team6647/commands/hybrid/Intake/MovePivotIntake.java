/**
 * Written by Juan Pablo Gutiérrez 
 * 02 09 2023
 */

package com.team6647.commands.hybrid.Intake;

import com.team6647.subsystems.IntakePivotSubsystem;
import com.team6647.subsystems.IntakePivotSubsystem.PivotState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Toggles the intake pivot between extended and homed.
 */
public class MovePivotIntake extends InstantCommand {
  private IntakePivotSubsystem cubeintakeSubsystem;
  private PivotState pivotState;

  /** Creates a new ToggleIntake. */
  public MovePivotIntake(IntakePivotSubsystem cubeintakeSubsystem, PivotState newState) {
    this.cubeintakeSubsystem = cubeintakeSubsystem;

    this.pivotState = newState;

    addRequirements(cubeintakeSubsystem);
  }

  @Override
  public void initialize() {
    cubeintakeSubsystem.resetPID();

    cubeintakeSubsystem.changePivotState(pivotState);
  }
}
