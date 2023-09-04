// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6647.commands.hybrid.elevator;

import com.team6647.subsystems.ElevatorSubsystem;
import com.team6647.subsystems.ElevatorSubsystem.ElevatorState;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendElevator extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;
  private ElevatorState elevatorState;

  public ExtendElevator(ElevatorSubsystem elevatorSubsystem, ElevatorState elevatorState) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.elevatorState = elevatorState;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.changeElevatorState(elevatorState);
  }
}
