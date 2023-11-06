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
import com.team6647.commands.hybrid.Intake.MovePivotIntake;
import com.team6647.commands.hybrid.arm.MoveArm;
import com.team6647.commands.hybrid.arm.MoveArmIntake;
import com.team6647.commands.hybrid.elevator.MoveElevator;
import com.team6647.subsystems.ArmIntakeSubsytem;
import com.team6647.subsystems.ArmPivotSubsystem;
import com.team6647.subsystems.AutoDriveSubsystem;
import com.team6647.subsystems.ElevatorSubsystem;
import com.team6647.subsystems.IntakePivotSubsystem;
import com.team6647.subsystems.VisionSubsystem;
import com.team6647.subsystems.ArmPivotSubsystem.ArmPivotState;
import com.team6647.subsystems.ElevatorSubsystem.ElevatorPositionState;
import com.team6647.subsystems.IntakePivotSubsystem.PivotState;
import com.team6647.subsystems.IntakeSubsystem.RollerState;
import com.team6647.util.AutoUtils;
import com.team6647.util.Constants.OperatorConstants;
import com.team6647.util.shuffleboard.TelemetryManager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer extends SuperRobotContainer {
        private static RobotContainer instance;

        /* Systems */
        private AndromedaSwerve andromedaSwerve;
        private AutoDriveSubsystem autoDriveSubsystem;
        private IntakePivotSubsystem cubeintakeSubsystem;
        private ElevatorSubsystem elevatorSubsystem;
        private ArmIntakeSubsytem armIntakeSubsystem;
        private ArmPivotSubsystem armPivotSubsystem;
        private VisionSubsystem visionSubsystem;

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
                                                AndromedaProfileConfig
                                                                .getConfig(AndromedaProfiles.ANDROMEDA_CONFIG)), },
                                AndromedaProfileConfig.getConfig(AndromedaProfiles.ANDROMEDA_CONFIG));

                autoDriveSubsystem = AutoDriveSubsystem.getInstance(andromedaSwerve);
                cubeintakeSubsystem = IntakePivotSubsystem.getInstance();
                elevatorSubsystem = ElevatorSubsystem.getInstance();
                armIntakeSubsystem = ArmIntakeSubsytem.getInstance();
                armPivotSubsystem = ArmPivotSubsystem.getInstance();
                visionSubsystem = VisionSubsystem.getInstance();

                /* Removes unused variable warning */
                autoDriveSubsystem.getClass();
                armIntakeSubsystem.getClass();
                visionSubsystem.getClass();
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

                OperatorConstants.driverController1.a()
                                .whileTrue(new RunCommand(() -> andromedaSwerve.resetNavx(), andromedaSwerve));

                /* Driver Controller 2 */

                /* Bottom Cone */
                OperatorConstants.driverController2.povLeft().and(OperatorConstants.driverController2.a())
                                .whileTrue(AutoUtils.teleopPlaceConeSequence(ElevatorPositionState.BOTTOM,
                                                RollerState.COLLECTING))
                                .onFalse(AutoUtils.teleopHomeSequence());
                /* Bottom Cube */
                OperatorConstants.driverController2.povLeft().and(OperatorConstants.driverController2.b())
                                .whileTrue(AutoUtils.teleopPlaceConeSequence(ElevatorPositionState.BOTTOM,
                                                RollerState.SPITTING))
                                .onFalse(AutoUtils.teleopHomeSequence());
                /* Middle Cone */
                OperatorConstants.driverController2.povRight().and(OperatorConstants.driverController2.a())
                                .whileTrue(AutoUtils.teleopPlaceConeSequence(ElevatorPositionState.MID_CONE,
                                                RollerState.COLLECTING))
                                .onFalse(AutoUtils.teleopHomeSequence());
                /* Middle Cube */
                OperatorConstants.driverController2.povRight().and(OperatorConstants.driverController2.b())
                                .whileTrue(AutoUtils.teleopPlaceConeSequence(ElevatorPositionState.MID,
                                                RollerState.SPITTING))
                                .onFalse(AutoUtils.teleopHomeSequence());

                /* Max Cone */
                OperatorConstants.driverController2.povUp().and(OperatorConstants.driverController2.a())
                                .whileTrue(AutoUtils.teleopPlaceConeSequence(ElevatorPositionState.MAX_CONE,
                                                RollerState.COLLECTING))
                                .onFalse(AutoUtils.teleopHomeSequence());

                /* Max Cube */
                OperatorConstants.driverController2.povUp().and(OperatorConstants.driverController2.b()) // Cube
                                .whileTrue(AutoUtils.teleopPlaceConeSequence(ElevatorPositionState.MAX,
                                                RollerState.SPITTING))
                                .onFalse(AutoUtils.teleopHomeSequence());

                /* Homes elevator */
                OperatorConstants.driverController2.povDown()
                                .whileTrue(new MoveElevator(elevatorSubsystem, armPivotSubsystem,
                                                ElevatorPositionState.FlOOR));

                /* Human Player */
                OperatorConstants.driverController2.leftBumper().whileTrue(Commands.sequence(
                                new MoveElevator(elevatorSubsystem, armPivotSubsystem,
                                                ElevatorPositionState.HUMAN_PLAYER),
                                new MoveArm(armPivotSubsystem, ArmPivotState.HUMAN_PLAYER).alongWith(
                                                new MoveArmIntake(armIntakeSubsystem, RollerState.SPITTING))))
                                .onFalse(Commands.sequence(
                                                new MoveArmIntake(armIntakeSubsystem,
                                                                RollerState.CONE_STOPPED),
                                                new MoveElevator(elevatorSubsystem, armPivotSubsystem,
                                                                ElevatorPositionState.HOMED)))
                                .and(OperatorConstants.driverController2.rightStick())
                                .whileTrue(new InstantCommand(() -> elevatorSubsystem.manualMovement(0.5)))
                                .and(OperatorConstants.driverController2.leftBumper())
                                .whileTrue(new InstantCommand(() -> elevatorSubsystem.manualMovement(-0.5)));

                OperatorConstants.driverController2.rightBumper().whileTrue(Commands.sequence(
                                new MoveElevator(elevatorSubsystem, armPivotSubsystem,
                                                ElevatorPositionState.HUMAN_PLAYER),
                                new MoveArm(armPivotSubsystem, ArmPivotState.HUMAN_PLAYER).alongWith(
                                                new MoveArmIntake(armIntakeSubsystem, RollerState.COLLECTING))))
                                .onFalse(Commands.sequence(
                                                new MoveArm(armPivotSubsystem, ArmPivotState.HOMED),
                                                new MoveArmIntake(armIntakeSubsystem,
                                                                RollerState.CONE_STOPPED),
                                                Commands.waitSeconds(0.5),
                                                new MoveElevator(elevatorSubsystem, armPivotSubsystem,
                                                                ElevatorPositionState.HOMED)))
                                .and(OperatorConstants.driverController2.rightStick())
                                .whileTrue(new InstantCommand(() -> elevatorSubsystem.manualMovement(0.5)))
                                .and(OperatorConstants.driverController2.leftBumper())
                                .whileTrue(new InstantCommand(() -> elevatorSubsystem.manualMovement(-0.5)));

                OperatorConstants.driverController2.leftStick()
                                .whileTrue(new InstantCommand(() -> elevatorSubsystem.manualMovement(0.5)));
                OperatorConstants.driverController2.rightStick()
                                .whileTrue(new InstantCommand(() -> elevatorSubsystem.manualMovement(-0.5)));

                /* Intake Collecting */

                OperatorConstants.driverController2.x().and(OperatorConstants.driverController2.leftTrigger())
                                .whileTrue(Commands
                                                .sequence(new MovePivotIntake(cubeintakeSubsystem, PivotState.EXTENDED),
                                                                AutoUtils.teleopIntakeCubeSequence(
                                                                                RollerState.COLLECTING)))
                                .onFalse(Commands.sequence(
                                                new MoveArm(armPivotSubsystem, ArmPivotState.HOMED),
                                                new MoveArmIntake(armIntakeSubsystem, RollerState.CUBE_STOPPED),
                                                new MovePivotIntake(cubeintakeSubsystem, PivotState.HOMED)));

                /* Intake Spitting */

                OperatorConstants.driverController2.x().and(OperatorConstants.driverController2.rightTrigger())
                                .whileTrue(Commands
                                                .sequence(new MovePivotIntake(cubeintakeSubsystem, PivotState.EXTENDED),
                                                                Commands.waitSeconds(0.4),
                                                                AutoUtils.teleopIntakeCubeSequence(
                                                                                RollerState.SPITTING)))
                                .onFalse(Commands.sequence(
                                                new MoveArm(armPivotSubsystem, ArmPivotState.HOMED),
                                                new MoveArmIntake(armIntakeSubsystem, RollerState.STOPPED),
                                                new MovePivotIntake(cubeintakeSubsystem, PivotState.HOMED)));

                /* Floor Cone Intake  */
                OperatorConstants.driverController2.y()
                                .whileTrue(AutoUtils.teleopConeIntakeSequence())
                                .onFalse(new MoveArm(armPivotSubsystem, ArmPivotState.HOMED)
                                                .alongWith(new MoveArmIntake(armIntakeSubsystem,
                                                                RollerState.CONE_STOPPED)));

        }

        public Command getAutonomousCommand() {
                try {
                        TelemetryManager.getInstance().setEventMap();
                        return AutoUtils.getAuto();
                } catch (Exception e) {
                        return Commands.waitSeconds(0.1);
                }

        }

}
