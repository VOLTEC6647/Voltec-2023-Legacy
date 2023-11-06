/**
 * Written by Juan Pablo Gutiérrez
 * 04 09 2023
 */

package com.team6647.util;

import com.team6647.commands.auto.AutonomousPaths;
import com.team6647.commands.hybrid.Intake.MoveIntake;
import com.team6647.commands.hybrid.arm.MoveArm;
import com.team6647.commands.hybrid.arm.MoveArmIntake;
import com.team6647.commands.hybrid.elevator.MoveElevator;
import com.team6647.subsystems.ArmIntakeSubsytem;
import com.team6647.subsystems.ArmPivotSubsystem;
import com.team6647.subsystems.AutoDriveSubsystem;
import com.team6647.subsystems.ElevatorSubsystem;
import com.team6647.subsystems.IntakePivotSubsystem;
import com.team6647.subsystems.IntakeSubsystem;
import com.team6647.subsystems.ArmPivotSubsystem.ArmPivotState;
import com.team6647.subsystems.ElevatorSubsystem.ElevatorPositionState;
import com.team6647.subsystems.IntakeSubsystem.RollerState;
import com.team6647.util.shuffleboard.TelemetryManager;
import com.team6647.util.shuffleboard.AutoModeSelector.AutoSelection;
import com.team6647.util.shuffleboard.GridPlacementSelector.GridPlacement;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class AutoUtils {

    protected static final TelemetryManager telemetryManager = TelemetryManager.getInstance();

    protected static IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    protected static ArmIntakeSubsytem armIntakeSubsystem = ArmIntakeSubsytem.getInstance();
    protected static ArmPivotSubsystem armPivotSubsystem = ArmPivotSubsystem.getInstance();
    protected static IntakePivotSubsystem intakePivotSubsystem = IntakePivotSubsystem.getInstance();
    protected static ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
    protected static AutoDriveSubsystem autoDriveSubsystem = AutoDriveSubsystem.getInstance(null);

    public enum Piece {
        Cone,
        Cube
    }

    public static Command getAuto() {
        AutoSelection selection = telemetryManager.getAutoSelection();

        if (selection == null)
            return null;

        switch (selection) {
            case Top:
                return AutonomousPaths.topAuto();
            case Middle:
                return AutonomousPaths.middleAuto();
            case Bottom:
                return AutonomousPaths.bottomAuto();
            case DoNothimg:
                return Commands.waitSeconds(0.1);
            case LeaveCommunity:
                return AutonomousPaths.leaveCommunityAuto();
            case OnlyPieceCone:
                return getGridPlacement(Piece.Cone);
            case OnlyPieceCube:
                return getGridPlacement(Piece.Cube);
            default:
                return Commands.waitSeconds(0.1);
        }
    }

    protected static Command getGridPlacement(Piece piece) {
        GridPlacement placement = telemetryManager.getGridPlacement();

        if (placement == null)
            return Commands.waitSeconds(0.1);

        System.out.println("Placement: " + placement);
        switch (placement) {
            case Bottom:
                if (piece == Piece.Cone)
                    return autoPlaceConeSequence(ElevatorPositionState.BOTTOM);

                if (piece == Piece.Cube)
                    return autoPlaceCubeSequence(ElevatorPositionState.BOTTOM);
            case Middle:
                if (piece == Piece.Cone)
                    return autoPlaceConeSequence(ElevatorPositionState.MID);

                if (piece == Piece.Cube)
                    return autoPlaceCubeSequence(ElevatorPositionState.MID);
            case Top:
                if (piece == Piece.Cone)
                    return autoPlaceConeSequence(ElevatorPositionState.MAX);

                if (piece == Piece.Cube)
                    return autoPlaceCubeSequence(ElevatorPositionState.MAX);
        }

        return null;
    }

    /* Teleop */
    public static Command teleopIntakeCubeSequence(RollerState rollerState) {
        return Commands.sequence(new MoveElevator(elevatorSubsystem, armPivotSubsystem, ElevatorPositionState.FlOOR),
                Commands.parallel(new MoveIntake(intakeSubsystem, rollerState),
                        Commands.print("AAAA"),
                        new MoveArm(armPivotSubsystem, ArmPivotState.INDEXING),
                        new MoveArmIntake(armIntakeSubsystem, rollerState)));
    }

    /* Teleop */
    public static Command teleopSpitCubeSequence(RollerState rollerState) {
        return Commands.sequence(new MoveElevator(elevatorSubsystem, armPivotSubsystem, ElevatorPositionState.FlOOR),
                new MoveArm(armPivotSubsystem, ArmPivotState.INDEXING),
                Commands.parallel(new MoveIntake(intakeSubsystem, rollerState),
                        Commands.print("AAAA"),
                        new MoveArmIntake(armIntakeSubsystem, rollerState)));
    }

    /* Spits the cone for intaking due to the inversed pulley system */
    public static Command teleopConeIntakeSequence() {
        return Commands.sequence(new MoveElevator(elevatorSubsystem, armPivotSubsystem, ElevatorPositionState.FlOOR),
                new MoveArm(armPivotSubsystem, ArmPivotState.FLOOR),
                new MoveArmIntake(armIntakeSubsystem, RollerState.SPITTING));
    }

    public static Command teleopHomeSequence() {
        return Commands.sequence(
                new MoveArmIntake(armIntakeSubsystem, RollerState.STOPPED).withTimeout(0.1),
                Commands.waitSeconds(0.5),
                new MoveElevator(elevatorSubsystem, armPivotSubsystem, ElevatorPositionState.HOMED));
    }

    public static Command teleopPlaceConeSequence(ElevatorPositionState elevatorPosition, RollerState rollerState) {
        return Commands.sequence(
                new MoveElevator(elevatorSubsystem, armPivotSubsystem, elevatorPosition),
                new MoveArm(armPivotSubsystem, ArmPivotState.SCORING),
                Commands.waitSeconds(0.1),
                Commands.sequence(
                        new MoveArm(armPivotSubsystem, ArmPivotState.PLACING),
                        Commands.waitSeconds(0.5),
                        new MoveArmIntake(armIntakeSubsystem,
                                rollerState)));
    }

    /* Auto */

    /* Spits the cone for intaking due to the inversed pulley system */
    public static Command autoIntakeConeSequence() {
        return new MoveElevator(elevatorSubsystem, armPivotSubsystem, ElevatorPositionState.FlOOR)
                .andThen(new MoveArm(armPivotSubsystem, ArmPivotState.FLOOR))
                .andThen(new RunCommand(() -> armIntakeSubsystem.changeRollerState(RollerState.SPITTING),
                        armIntakeSubsystem));
    }

    public static Command autoPlaceConeSequence(ElevatorPositionState elevatorState) {
        return Commands.sequence(
                new MoveElevator(elevatorSubsystem, armPivotSubsystem, elevatorState),
                new MoveArm(armPivotSubsystem, ArmPivotState.SCORING),
                Commands.waitSeconds(0.7),
                Commands.sequence(
                        new MoveArm(armPivotSubsystem, ArmPivotState.PLACING),
                        Commands.waitSeconds(0.5),
                        new MoveArmIntake(armIntakeSubsystem, RollerState.COLLECTING)).withTimeout(2),
                teleopHomeSequence());
    }

    public static Command autoPlaceCubeSequence(ElevatorPositionState elevatorState) {
        return Commands.sequence(new MoveElevator(elevatorSubsystem, armPivotSubsystem, elevatorState),
                new MoveArm(armPivotSubsystem, ArmPivotState.SCORING),
                new MoveArm(armPivotSubsystem, ArmPivotState.PLACING),
                new MoveArmIntake(armIntakeSubsystem, RollerState.SPITTING),
                Commands.waitSeconds(1.5),
                teleopHomeSequence());
    }

}
