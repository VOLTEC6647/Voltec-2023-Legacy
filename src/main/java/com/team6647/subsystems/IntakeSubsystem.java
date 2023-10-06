/**
 * Written by Juan Pablo Gutiérrez
 * 02 09 2023
 */
package com.team6647.subsystems;

import com.andromedalib.motorControllers.SuperSparkMax;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.team6647.util.Constants.IntakeConstants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private static IntakeSubsystem instance;
  private static NetworkTable intakeRollerTable;
  private static StringEntry intakeRollerPublisher;

  private SuperSparkMax intakeMotor = new SuperSparkMax(IntakeConstants.intakeMotorID, GlobalIdleMode.Coast, true, 80);

  private RollerState mState = RollerState.STOPPED;

  /** Creates a new IntakeSubsystem. */
  private IntakeSubsystem() {
    intakeRollerTable = NetworkTableInstance.getDefault().getTable("IntakeTable/Roller");
    intakeRollerPublisher = intakeRollerTable.getStringTopic("IntakeRollerState").getEntry(getRollerState().toString());
  }

  public static IntakeSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakeSubsystem();
    }
    return instance;
  }

  public enum RollerState {
    STOPPED, COLLECTING, SPITTING
  }

  @Override
  public void periodic() {
    updateNT();
  }

  public void changeRollerState(RollerState rollerState) {
    switch (rollerState) {
      case STOPPED:
        mState = RollerState.STOPPED;
        setIntakeSpeed(0);
        break;
      case COLLECTING:
        mState = RollerState.COLLECTING;
        setIntakeSpeed(IntakeConstants.intakeSpeed);
        break;
      case SPITTING:
        mState = RollerState.SPITTING;
        setIntakeSpeed(-IntakeConstants.intakeSpeed);
        break;
    }
  }

  private void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public RollerState getRollerState() {
    return mState;
  }

  /* Telemetry */
  private void updateNT() {
    intakeRollerPublisher.set(mState.toString());
  }
}
