// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6647.subsystems;

import com.andromedalib.andromedaSwerve.systems.AndromedaSwerve;
import com.andromedalib.andromedaSwerve.utils.SwerveConstants;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoDriveSubsystem extends SubsystemBase {

  private static AutoDriveSubsystem instance;

  AndromedaSwerve swerve;

  SwerveDriveOdometry odometry;

  Field2d field;

  /** Creates a new AutoDriveSubsystem. */
  private AutoDriveSubsystem(AndromedaSwerve swerve) {
    this.swerve = swerve;

    field = new Field2d();

    this.odometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, swerve.getAngle(), swerve.getPositions());
  }

  public static AutoDriveSubsystem getInstance(AndromedaSwerve swerve) {
    if (instance == null) {
      instance = new AutoDriveSubsystem(swerve);
    }
    return instance;
  }

  @Override
  public void periodic() {
    odometry.update(swerve.getAngle(), swerve.getPositions());
    field.setRobotPose(getPose());
    SmartDashboard.putData(field);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(swerve.getAngle(), swerve.getPositions(), pose);
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory trajectory, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          if (isFirstPath) {
            this.resetOdometry(trajectory.getInitialHolonomicPose());
          }
        }),

        new PPSwerveControllerCommand(
            trajectory,
            this::getPose,
            SwerveConstants.swerveKinematics,
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0),
            new PIDController(1.75, 0, 0),
            swerve::setModuleStates,
            true,
            swerve));

  }
}
