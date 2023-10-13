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
import com.team6647.commands.hybrid.Intake.ToggleIntake;
import com.team6647.commands.hybrid.arm.MoveArm;
import com.team6647.commands.hybrid.arm.MoveArmIntake;
import com.team6647.commands.hybrid.elevator.ExtendElevator;
import com.team6647.subsystems.ArmIntakeSubsytem;
import com.team6647.subsystems.ArmPivotSubsystem;
import com.team6647.subsystems.AutoDriveSubsystem;
import com.team6647.subsystems.ElevatorSubsystem;
import com.team6647.subsystems.IntakePivotSubsystem;
import com.team6647.subsystems.ArmPivotSubsystem.ArmPivotState;
import com.team6647.subsystems.ElevatorSubsystem.ElevatorPositionState;
import com.team6647.subsystems.IntakeSubsystem.RollerState;
import com.team6647.util.AutoUtils;
import com.team6647.util.Constants.OperatorConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer extends SuperRobotContainer {
    private static RobotContainer instance;

    /* Systems */
    private AndromedaSwerve andromedaSwerve;
    private AutoDriveSubsystem autoDriveSubsystem;
    private IntakePivotSubsystem cubeintakeSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private ArmIntakeSubsytem armIntakeSubsystem;
    private ArmPivotSubsystem armPivotSubsystem;

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
                new FalconAndromedaModule(0, "Front Right Module", AndromedaMap.mod1Const,
                        AndromedaProfileConfig.getConfig(AndromedaProfiles.ANDROMEDA_CONFIG)),
                new FalconAndromedaModule(1, "Back Right Module", AndromedaMap.mod2Const,
                        AndromedaProfileConfig.getConfig(AndromedaProfiles.ANDROMEDA_CONFIG)),
                new FalconAndromedaModule(2, "Back Left Module", AndromedaMap.mod3Const,
                        AndromedaProfileConfig.getConfig(AndromedaProfiles.ANDROMEDA_CONFIG)),
                new FalconAndromedaModule(3, "Front Left Module", AndromedaMap.mod4Const,
                        AndromedaProfileConfig.getConfig(AndromedaProfiles.ANDROMEDA_CONFIG)), },
                AndromedaProfileConfig.getConfig(AndromedaProfiles.ANDROMEDA_CONFIG));

        autoDriveSubsystem = AutoDriveSubsystem.getInstance(andromedaSwerve);
        cubeintakeSubsystem = IntakePivotSubsystem.getInstance();
        elevatorSubsystem = ElevatorSubsystem.getInstance();
        armIntakeSubsystem = ArmIntakeSubsytem.getInstance();
        armPivotSubsystem = ArmPivotSubsystem.getInstance();
    }

    @Override
    public void configureBindings() {

        /* Driver Controller 1 */
        andromedaSwerve.setDefaultCommand(
                new SwerveDriveCommand(
                        andromedaSwerve,
                        () -> -OperatorConstants.driverController1.getLeftX(),
                        () -> -OperatorConstants.driverController1.getLeftY(),
                        () -> -OperatorConstants.driverController1.getRightX(),
                        () -> OperatorConstants.driverController1.leftStick().getAsBoolean()));

        /* Driver Controller 2 */

        /* Testing Bidings */

        OperatorConstants.driverController2.povLeft()
                .whileTrue(new ExtendElevator(elevatorSubsystem, ElevatorPositionState.BOTTOM))
                .and(OperatorConstants.driverController2.a()
                        .whileTrue(Commands.parallel(new MoveArm(armPivotSubsystem, ArmPivotState.PLACING),
                                new MoveArmIntake(armIntakeSubsystem, RollerState.SPITTING))))
                .onFalse(Commands.parallel(new ExtendElevator(elevatorSubsystem, ElevatorPositionState.HOMED),
                        new MoveArm(armPivotSubsystem, ArmPivotState.HOMED)));
        OperatorConstants.driverController2.povRight()
                .whileTrue(new ExtendElevator(elevatorSubsystem, ElevatorPositionState.MID))
                .and(OperatorConstants.driverController2.a()
                        .whileTrue(Commands.parallel(new MoveArm(armPivotSubsystem, ArmPivotState.PLACING),
                                new MoveArmIntake(armIntakeSubsystem, RollerState.SPITTING))))
                .onFalse(Commands.parallel(new ExtendElevator(elevatorSubsystem, ElevatorPositionState.HOMED),
                        new MoveArm(armPivotSubsystem, ArmPivotState.HOMED)));
        OperatorConstants.driverController2.povUp()
                .whileTrue(new ExtendElevator(elevatorSubsystem, ElevatorPositionState.MAX))
                .and(OperatorConstants.driverController2.a()
                        .whileTrue(Commands.parallel(new MoveArm(armPivotSubsystem, ArmPivotState.PLACING),
                                new MoveArmIntake(armIntakeSubsystem, RollerState.SPITTING))))
                .onFalse(Commands.parallel(new ExtendElevator(elevatorSubsystem, ElevatorPositionState.HOMED),
                        new MoveArm(armPivotSubsystem, ArmPivotState.HOMED)));

        OperatorConstants.driverController2.povDown()
                .whileTrue(Commands.parallel(new ExtendElevator(elevatorSubsystem, ElevatorPositionState.MAX),
                        new MoveArmIntake(armIntakeSubsystem, RollerState.COLLECTING)))
                .onFalse(new ExtendElevator(elevatorSubsystem, ElevatorPositionState.HOMED));

        OperatorConstants.driverController2.leftTrigger()
                .whileTrue(new ExtendElevator(elevatorSubsystem, ElevatorPositionState.HOMED));

        OperatorConstants.driverController2.x().whileTrue(new ToggleIntake(cubeintakeSubsystem))
                .and(OperatorConstants.driverController2.leftTrigger()
                        .whileTrue(AutoUtils.intakePieceSequence(RollerState.COLLECTING))
                        .or(OperatorConstants.driverController2
                                .rightTrigger().whileTrue(AutoUtils.intakePieceSequence(RollerState.SPITTING))))
                .onFalse(Commands.parallel(new MoveArm(armPivotSubsystem, ArmPivotState.HOMED),
                        new ToggleIntake(cubeintakeSubsystem)));

    }

    public Command getAutonomousCommand() {
        return autoDriveSubsystem.createFullAuto("Top");

        // PathPlannerTrajectory examplePath = PathPlanner.loadPath("Top", new
        // PathConstraints(2, 2));
        // return autoDriveSubsystem.followTrajectoryCommand(examplePath, true);

    }

}
