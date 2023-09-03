// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6647.commands.hybrid.Intake;

import com.team6647.subsystems.IntakeSubsystem;
import com.team6647.subsystems.IntakeSubsystem.RollerState;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveIntake extends CommandBase {
  private IntakeSubsystem intakeSubsystem;
  private RollerState rollerState;
  /** Creates a new MoveIntake. */
  public MoveIntake(IntakeSubsystem intakeSubsystem, RollerState rollerState) {
    this.intakeSubsystem = intakeSubsystem;
    this.rollerState = rollerState;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.changeRollerState(rollerState);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.changeRollerState(RollerState.STOPPED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
