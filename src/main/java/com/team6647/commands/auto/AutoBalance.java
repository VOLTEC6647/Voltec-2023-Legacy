// Copyright (c) FIRST and other WPILib contributors.
/**
 * Written by Juan Pablo Gutiérrez
 * 04 09 2023
 */
package com.team6647.commands.auto;

import com.andromedalib.andromedaSwerve.systems.AndromedaSwerve;
import com.team6647.subsystems.AutoDriveSubsystem;
import com.team6647.util.Constants.DriveConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalance extends CommandBase {
  private AndromedaSwerve swerve;
  private AutoDriveSubsystem autoDriveSubsystem;

  double drivePower, currentAngle, error = 0;

  public AutoBalance(AndromedaSwerve swerve, AutoDriveSubsystem autoDriveSubsystem) {
    this.swerve = swerve;
    this.autoDriveSubsystem = autoDriveSubsystem;

    addRequirements(swerve, autoDriveSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    currentAngle = autoDriveSubsystem.getNavxRoll();
    error = DriveConstants.balanceGoal - currentAngle;

    drivePower = -Math.min(DriveConstants.balanceKp * error, 1);

    SmartDashboard.putNumber("Drivepower: ", drivePower);

    if (drivePower < 0) {
      drivePower *= 0.8;
    }

    if (Math.abs(drivePower) > 0.35) {
      drivePower = Math.copySign(0.35, drivePower);
    }
    swerve.drive(new Translation2d(drivePower, 0), currentAngle, false, true);

  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(1, 1), currentAngle, false, true);
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(error) < DriveConstants.balanceTolerance);
  }
}
