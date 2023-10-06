/**
 * Written by Juan Pablo Gutiérrez
 * 06 10 2023
 */
package com.team6647.commands.hybrid.arm;

import com.team6647.subsystems.ArmIntakeSubsytem;
import com.team6647.subsystems.IntakeSubsystem.RollerState;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Moves the arm intake roller to a specified state.
 */
public class MoveArmIntake extends CommandBase {
  private ArmIntakeSubsytem armIntakeSubsystem;
  private RollerState rollerState;
  /** Creates a new MoveArmIntake. */
  public MoveArmIntake(ArmIntakeSubsytem armIntakeSubsystem, RollerState rollerState) {
    this.armIntakeSubsystem = armIntakeSubsystem;
    this.rollerState = rollerState;

    addRequirements(armIntakeSubsystem); 
  }

  @Override
  public void initialize() {
    armIntakeSubsystem.changeRollerState(rollerState);
  }

  @Override
  public void end(boolean interrupted) {
    armIntakeSubsystem.changeRollerState(RollerState.STOPPED);
  }
}
