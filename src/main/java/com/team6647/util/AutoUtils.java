/**
 * Written by Juan Pablo Gutiérrez
 * 04 09 2023
 */

package com.team6647.util;

import com.team6647.commands.auto.AutonomousPaths;
import com.team6647.commands.hybrid.Intake.MoveIntake;
import com.team6647.commands.hybrid.arm.MoveArm;
import com.team6647.commands.hybrid.arm.MoveArmIntake;
import com.team6647.commands.hybrid.elevator.ExtendElevator;
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
            case OnlyPiece:
                return getGridPlacement(Piece.Cone);
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
                    return placeConeSequence(ElevatorPositionState.BOTTOM);

                if (piece == Piece.Cube)
                    return placeCubeSequence(ElevatorPositionState.BOTTOM);
            case Middle:
                if (piece == Piece.Cone)
                    return placeConeSequence(ElevatorPositionState.MID);

                if (piece == Piece.Cube)
                    return placeCubeSequence(ElevatorPositionState.MID);
            case Top:
                if (piece == Piece.Cone)
                    return placeConeSequence(ElevatorPositionState.MAX);

                if (piece == Piece.Cube)
                    return placeCubeSequence(ElevatorPositionState.MAX);
        }

        return null;
    }

    public static Command intakeCubeSequence(RollerState rollerState) {
        return Commands.parallel(new MoveIntake(intakeSubsystem, rollerState),
                new MoveArm(armPivotSubsystem, ArmPivotState.INDEXING),
                new MoveArmIntake(armIntakeSubsystem, rollerState));
    }

    public static Command intakeConeSequence() {
        return Commands.sequence(new MoveArm(armPivotSubsystem, ArmPivotState.FLOOR),
                new MoveArmIntake(armIntakeSubsystem, RollerState.COLLECTING).withTimeout(1),
                new MoveArm(armPivotSubsystem, ArmPivotState.HOMED));
    }

    public static Command placeConeSequence(ElevatorPositionState elevatorState) {
        return Commands.sequence(new ExtendElevator(elevatorSubsystem, elevatorState),
                Commands.parallel(new MoveArm(armPivotSubsystem, ArmPivotState.SCORING),
                        new MoveArmIntake(armIntakeSubsystem, RollerState.SPITTING)).withTimeout(2));
    }

    public static Command placeCubeSequence(ElevatorPositionState elevatorState) {
        return Commands.sequence(new ExtendElevator(elevatorSubsystem, elevatorState),
                Commands.parallel(
                        new MoveArmIntake(armIntakeSubsystem, RollerState.SPITTING)).withTimeout(2));
    }

}
