/**
 * Written by Juan Pablo Gutiérrez
 * 03 09 2023
 */

package com.team6647.subsystems;

import com.andromedalib.motorControllers.SuperSparkMax;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.team6647.util.Constants.IndexerConstants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
  private static IndexerSubsystem instance;
  private static NetworkTable indexerTable;
  private static StringPublisher indexerStatePublisher;

  private static SuperSparkMax indexerMotor = new SuperSparkMax(IndexerConstants.indexerMotorID, GlobalIdleMode.Coast,
      true, 80);

  private IndexerState mState = IndexerState.STOPPED;

  private IndexerSubsystem() {
    indexerTable = NetworkTableInstance.getDefault().getTable("IndexerTable");
    indexerStatePublisher = indexerTable.getStringTopic("IndexerState").publish();
  }

  @Override
  public void periodic() {
    updateNT();
  }

  public static IndexerSubsystem getInstance() {
    if (instance == null)
      instance = new IndexerSubsystem();
    return instance;
  }

  public enum IndexerState {
    STOPPED, INDEXING, SPITTING
  }

  public void changeIndexerState(IndexerState newState) {
    switch (newState) {
      case STOPPED:
        mState = IndexerState.STOPPED;
        setIndexer(0);
        break;
      case INDEXING:
        mState = IndexerState.INDEXING;
        setIndexer(IndexerConstants.indexerSpeed);
        break;
      case SPITTING:
        mState = IndexerState.SPITTING;
        setIndexer(-IndexerConstants.indexerSpeed);
        break;
    }
  }

  private void setIndexer(double speed) {
    indexerMotor.set(speed);
  }

  /* Telemetry */

  private void updateNT() {
    indexerStatePublisher.set(mState.toString());
  }
}
