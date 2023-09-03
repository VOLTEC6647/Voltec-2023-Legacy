/**
 * Written by Juan Pablo Gutiérrez
 * 02 09 2023
 */
package com.team6647.subsystems;

import com.andromedalib.motorControllers.SuperSparkMax;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.team6647.util.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private static IntakeSubsystem instance;

  private SuperSparkMax intakeMotor = new SuperSparkMax(IntakeConstants.intakeMotorID, GlobalIdleMode.Coast, false, 80);

  private RollerState mState = RollerState.STOPPED;

  /** Creates a new IntakeSubsystem. */
  private IntakeSubsystem() {
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

  /* Telemetry */
  public RollerState getRollerState(){
    return mState;
  }
}
