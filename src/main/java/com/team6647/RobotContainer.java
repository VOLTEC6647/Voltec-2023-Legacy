/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647;

import com.andromedalib.andromedaSwerve.andromedaModule.FalconAndromedaModule;
import com.andromedalib.andromedaSwerve.commands.SwerveDriveCommand;
import com.andromedalib.andromedaSwerve.systems.AndromedaSwerve;
import com.andromedalib.andromedaSwerve.utils.AndromedaMap;
import com.andromedalib.andromedaSwerve.utils.AndromedaProfileConfig;
import com.andromedalib.andromedaSwerve.utils.AndromedaProfileConfig.AndromedaProfiles;
import com.andromedalib.robot.SuperRobotContainer;
import com.team6647.commands.hybrid.Intake.IntakePieceSequence;
import com.team6647.commands.hybrid.Intake.ToggleIntake;
import com.team6647.subsystems.AutoDriveSubsystem;
import com.team6647.subsystems.IndexerSubsystem;
import com.team6647.subsystems.IntakeSubsystem;
import com.team6647.subsystems.PivotCubeSubsystem;
import com.team6647.subsystems.IndexerSubsystem.IndexerState;
import com.team6647.subsystems.IntakeSubsystem.RollerState;
import com.team6647.util.Constants.OperatorConstants;

import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer extends SuperRobotContainer {
  private static RobotContainer instance;

  /* Systems */
  private AndromedaSwerve andromedaSwerve;
  private AutoDriveSubsystem autoDriveSubsystem;
  private PivotCubeSubsystem cubeintakeSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private IndexerSubsystem indexerSubsystem;
  // private ElevatorSubsystem elevatorSubsystem;

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
        new FalconAndromedaModule(3, "Front Lert Module", AndromedaMap.mod4Const), }, AndromedaProfileConfig.getConfig(AndromedaProfiles.ANDROMEDA_CONFIG));

    autoDriveSubsystem = AutoDriveSubsystem.getInstance(andromedaSwerve);
    intakeSubsystem = IntakeSubsystem.getInstance();
    cubeintakeSubsystem = PivotCubeSubsystem.getInstance();
    indexerSubsystem = IndexerSubsystem.getInstance();
    // elevatorSubsystem = ElevatorSubsystem.getInstance();
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

    OperatorConstants.driverController1.leftTrigger()
        .whileTrue(
            new IntakePieceSequence(intakeSubsystem, indexerSubsystem, RollerState.COLLECTING, IndexerState.INDEXING));

    OperatorConstants.driverController1.rightTrigger()
        .whileTrue(
            new IntakePieceSequence(intakeSubsystem, indexerSubsystem, RollerState.SPITTING, IndexerState.SPITTING));

    OperatorConstants.driverController1.x().whileTrue(new ToggleIntake(cubeintakeSubsystem));

    // OperatorConstants.driverController2.a().whileTrue(new InstantCommand(() ->
    // elevatorSubsystem.changeElevatorState(ElevatorState.MAX)));
    // OperatorConstants.driverController2.b().whileTrue(new RunCommand(() ->
    // elevatorSubsystem.moveElevator(-0.3)));

  }

  public Command getAutonomousCommand() {
    // return null;
    return autoDriveSubsystem.createFullAuto("Top");

    //PathPlannerTrajectory examplePath = PathPlanner.loadPath("Top", new PathConstraints(2, 2));
    //return autoDriveSubsystem.followTrajectoryCommand(examplePath, true);

  }

}
