// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
