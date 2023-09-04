/**
 * Written by Juan Pablo Gutiérrez
 * 03 09 2023
 */
package com.team6647.commands.hybrid.Intake;

import com.team6647.subsystems.IndexerSubsystem;
import com.team6647.subsystems.IntakeSubsystem;
import com.team6647.subsystems.IndexerSubsystem.IndexerState;
import com.team6647.subsystems.IntakeSubsystem.RollerState;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class IntakePieceSequence extends ParallelCommandGroup {

  public IntakePieceSequence(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexer) {

    addCommands(new MoveIntake(intakeSubsystem, RollerState.COLLECTING),
        new MoveIndexer(indexer, IndexerState.INDEXING));
  }
}
