/**
 * Written by Juan Pablo Gutiérrez
 * 04 09 2023
 */

package com.team6647.util;

import com.andromedalib.andromedaSwerve.systems.AndromedaSwerve;
import com.team6647.commands.hybrid.Intake.MoveIntake;
import com.team6647.commands.hybrid.arm.MoveArm;
import com.team6647.commands.hybrid.arm.MoveArmIntake;
import com.team6647.subsystems.ArmIntakeSubsytem;
import com.team6647.subsystems.ArmPivotSubsystem;
import com.team6647.subsystems.ElevatorSubsystem;
import com.team6647.subsystems.IntakePivotSubsystem;
import com.team6647.subsystems.IntakeSubsystem;
import com.team6647.subsystems.ArmPivotSubsystem.ArmPivotState;
import com.team6647.subsystems.IntakeSubsystem.RollerState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoUtils {
/*     private static IntakePivotSubsystem cubeintakeSubsystem = IntakePivotSubsystem.getInstance();
    private static IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private static IndexerSubsystem indexerSubsystem = IndexerSubsystem.getInstance();
    private static ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
    private static ArmIntakeSubsytem armIntakeSubsystem = ArmIntakeSubsytem.getInstance();
    private static ArmPivotSubsystem armPivotSubsystem = ArmPivotSubsystem.getInstance();

    public static Command intakePieceSequence(RollerState rollerState, IndexerState indexerState) {
        return Commands.parallel(new MoveIntake(intakeSubsystem, rollerState),
                new MoveArm(armPivotSubsystem, ArmPivotState.INDEXING),
                new MoveIndexer(indexerSubsystem, indexerState), new MoveArmIntake(armIntakeSubsystem, rollerState));
    } */
}
