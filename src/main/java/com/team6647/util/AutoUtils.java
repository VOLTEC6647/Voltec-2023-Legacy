/**
 * Written by Juan Pablo Gutiérrez
 * 04 09 2023
 */

package com.team6647.util;

import com.team6647.commands.hybrid.Intake.MoveIndexer;
import com.team6647.commands.hybrid.Intake.MoveIntake;
import com.team6647.subsystems.IndexerSubsystem;
import com.team6647.subsystems.IntakeSubsystem;
import com.team6647.subsystems.IndexerSubsystem.IndexerState;
import com.team6647.subsystems.IntakeSubsystem.RollerState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoUtils {

    public static Command intakePieceSequence(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexer,
            RollerState rollerState, IndexerState indexerState) {
        return Commands.parallel(new MoveIntake(intakeSubsystem, rollerState), new MoveIndexer(indexer, indexerState));
    }
}
