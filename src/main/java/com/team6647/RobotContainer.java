// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6647;

import com.andromedalib.andromedaSwerve.andromedaModule.FalconAndromedaModule;
import com.andromedalib.andromedaSwerve.commands.SwerveDriveCommand;
import com.andromedalib.andromedaSwerve.systems.AndromedaSwerve;
import com.andromedalib.andromedaSwerve.utils.AndromedaMap;
import com.andromedalib.robot.SuperRobotContainer;
import com.team6647.commands.hybrid.Intake.MoveIntake;
import com.team6647.commands.hybrid.Intake.ToggleIntake;
import com.team6647.subsystems.IntakeSubsystem;
import com.team6647.subsystems.PivotCubeSubsystem;
import com.team6647.subsystems.IntakeSubsystem.RollerState;
import com.team6647.util.Constants.OperatorConstants;

import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer extends SuperRobotContainer {
  private static RobotContainer instance;

  /* Systems */
  private AndromedaSwerve andromedaSwerve;

  private PivotCubeSubsystem cubeintakeSubsystem;
  private IntakeSubsystem intakeSubsystem;

  private RobotContainer() {
  }

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }

    return instance;
  }

  @Override
  public void initSubsystems() {
    andromedaSwerve = AndromedaSwerve.getInstance(new FalconAndromedaModule[] {
        new FalconAndromedaModule(0, "Front Right Module", AndromedaMap.mod1Const),
        new FalconAndromedaModule(1, "Back Right Module", AndromedaMap.mod2Const),
        new FalconAndromedaModule(2, "Back Left Module", AndromedaMap.mod3Const),
        new FalconAndromedaModule(3, "Front Lert Module", AndromedaMap.mod4Const), });
    /*
     * autoDriveSubsystem = AutoDriveSubsystem.getInstance(andromedaSwerve);
     */
    cubeintakeSubsystem = PivotCubeSubsystem.getInstance();
    intakeSubsystem = IntakeSubsystem.getInstance();
  }

  @Override
  public void configureBindings() {
    andromedaSwerve.setDefaultCommand(
        new SwerveDriveCommand(
            andromedaSwerve,
            () -> -OperatorConstants.driverController1.getLeftX(),
            () -> -OperatorConstants.driverController1.getLeftY(),
            () -> -OperatorConstants.driverController1.getRightX(),
            () -> OperatorConstants.driverController1.leftStick().getAsBoolean()));

    OperatorConstants.driverController2.leftTrigger().whileTrue(new MoveIntake(intakeSubsystem, RollerState.COLLECTING));
    OperatorConstants.driverController2.rightTrigger().whileTrue(new MoveIntake(intakeSubsystem, RollerState.SPITTING));

    OperatorConstants.driverController2.x().onTrue(new ToggleIntake(cubeintakeSubsystem));
  }

  public Command getAutonomousCommand() {
    return null;
    /*
     * PathPlannerTrajectory examplePath = PathPlanner.loadPath("Straigth", new
     * PathConstraints(1, 1));
     * return autoDriveSubsystem.followTrajectoryCommand(examplePath, true);
     */
  }

}
