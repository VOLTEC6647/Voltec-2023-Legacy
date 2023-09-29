/**
 * Written by Juan Pablo Gutiérrez
 * 
 * 04 09 2023
 */
package com.team6647.commands.hybrid.Intake;

import com.team6647.subsystems.IndexerSubsystem;
import com.team6647.subsystems.IndexerSubsystem.IndexerState;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveIndexer extends CommandBase {
  private IndexerSubsystem indexerSubsystem;
  private IndexerState indexerState;

  public MoveIndexer(IndexerSubsystem indexerSubsystem, IndexerState indexerState) {
    this.indexerSubsystem = indexerSubsystem;
    this.indexerState = indexerState;
    
    addRequirements(indexerSubsystem);
  }

  @Override
  public void initialize() {
    indexerSubsystem.changeIndexerState(indexerState);
  }
  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.changeIndexerState(IndexerState.STOPPED);
  }
}
