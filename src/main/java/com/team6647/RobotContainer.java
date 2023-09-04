// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6647;

import com.andromedalib.andromedaSwerve.andromedaModule.FalconAndromedaModule;
import com.andromedalib.andromedaSwerve.commands.SwerveDriveCommand;
import com.andromedalib.andromedaSwerve.systems.AndromedaSwerve;
import com.andromedalib.andromedaSwerve.utils.AndromedaMap;
import com.andromedalib.robot.SuperRobotContainer;
import com.team6647.commands.hybrid.Intake.IntakePiece;
import com.team6647.commands.hybrid.Intake.MoveIntake;
import com.team6647.commands.hybrid.Intake.ToggleIntake;
import com.team6647.subsystems.AutoDriveSubsystem;
import com.team6647.subsystems.ElevatorSubsystem;
import com.team6647.subsystems.IndexerSubsystem;
import com.team6647.subsystems.IntakeSubsystem;
import com.team6647.subsystems.PivotCubeSubsystem;
import com.team6647.subsystems.ElevatorSubsystem.ElevatorStates;
import com.team6647.subsystems.IntakeSubsystem.RollerState;
import com.team6647.util.Constants.OperatorConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer extends SuperRobotContainer {
  private static RobotContainer instance;

  /* Systems */
  private AndromedaSwerve andromedaSwerve;
  private AutoDriveSubsystem autoDriveSubsystem;
  private PivotCubeSubsystem cubeintakeSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private IndexerSubsystem indexerSubsystem;
  private ElevatorSubsystem elevatorSubsystem;

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

    autoDriveSubsystem = AutoDriveSubsystem.getInstance(andromedaSwerve);
    intakeSubsystem = IntakeSubsystem.getInstance();
    cubeintakeSubsystem = PivotCubeSubsystem.getInstance();
    indexerSubsystem = IndexerSubsystem.getInstance();
    elevatorSubsystem = ElevatorSubsystem.getInstance();
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

    OperatorConstants.driverController2.leftTrigger()
        .whileTrue(new IntakePiece(intakeSubsystem, indexerSubsystem));
    OperatorConstants.driverController2.rightTrigger().whileTrue(new MoveIntake(intakeSubsystem, RollerState.SPITTING));

    OperatorConstants.driverController2.x().whileTrue(new ToggleIntake(cubeintakeSubsystem));

    OperatorConstants.driverController2.a().whileTrue(new InstantCommand(() -> elevatorSubsystem.changeElevatorState(ElevatorStates.MAX)));
  }

  public Command getAutonomousCommand() {
    // return null;
    return autoDriveSubsystem.createFullAuto();
    /*
     * PathPlannerTrajectory examplePath = PathPlanner.loadPath("Straigth", new
     * PathConstraints(1, 1));
     * return autoDriveSubsystem.followTrajectoryCommand(examplePath, true);
     */
  }

}
