// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6647.commands.hybrid.elevator;

import com.team6647.commands.hybrid.arm.MoveArm;
import com.team6647.subsystems.ArmPivotSubsystem;
import com.team6647.subsystems.ElevatorSubsystem;
import com.team6647.subsystems.ArmPivotSubsystem.ArmPivotState;
import com.team6647.subsystems.ElevatorSubsystem.ElevatorPositionState;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveElevator extends SequentialCommandGroup {
  /** Creates a new MoveElevator. */
  public MoveElevator(ElevatorSubsystem elevatorSubsystem, ArmPivotSubsystem armPivotSubsystem,
      ElevatorPositionState state) {

    addCommands(new MoveArm(armPivotSubsystem, ArmPivotState.HOMED).withTimeout(2), Commands.waitSeconds(0.1),
        new ExtendElevator(elevatorSubsystem, state));
  }
}
