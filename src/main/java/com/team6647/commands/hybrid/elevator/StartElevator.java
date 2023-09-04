/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647.commands.hybrid.elevator;

import com.team6647.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class StartElevator extends CommandBase {

  private ElevatorSubsystem elevatorSubsystem;

  public StartElevator(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize(){
    elevatorSubsystem.disablePID();
  }

  @Override
  public void execute() {
    elevatorSubsystem.moveElevator(-0.25);
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.moveElevator(0);
    elevatorSubsystem.resetElevatorPosition();
    elevatorSubsystem.enablePID();
  }

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.getLimitState();
  }
}
