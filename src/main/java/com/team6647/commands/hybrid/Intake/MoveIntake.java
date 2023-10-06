/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 02 09 2023
 */
package com.team6647.commands.hybrid.Intake;

import com.team6647.subsystems.IntakeSubsystem;
import com.team6647.subsystems.IntakeSubsystem.RollerState;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Moves the intake roller to a specified state.
 */
public class MoveIntake extends CommandBase {
  private IntakeSubsystem intakeSubsystem;
  private RollerState rollerState;

  public MoveIntake(IntakeSubsystem intakeSubsystem, RollerState rollerState) {
    this.intakeSubsystem = intakeSubsystem;
    this.rollerState = rollerState;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.changeRollerState(rollerState);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.changeRollerState(RollerState.STOPPED);
  }

}
