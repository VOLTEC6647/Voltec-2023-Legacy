/**
 * Written by Juan Pablo Gutiérrez
 * 28 - 08 - 2023
 */

package com.team6647.subsystems;

import com.andromedalib.motorControllers.SuperTalonFX;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team6647.subsystems.IntakeSubsystem.RollerState;
import com.team6647.util.Constants.ConeIntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConeIntakeSubsystem extends SubsystemBase {
  private static ConeIntakeSubsystem instance;

  private SuperTalonFX intakeMotor = new SuperTalonFX(ConeIntakeConstants.intakeMotorID, GlobalIdleMode.Coast, false);

  private RollerState mState = RollerState.STOPPED;

  /** Creates a new ConeIntakeSubsystem. */
  private ConeIntakeSubsystem() {
  }

  public static ConeIntakeSubsystem getInstance() {
    if (instance == null) {
      instance = new ConeIntakeSubsystem();
    }
    return instance;
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
        setIntakeSpeed(ConeIntakeConstants.intakeSpeed);
        break;
      case SPITTING:
        mState = RollerState.SPITTING;
        setIntakeSpeed(-ConeIntakeConstants.intakeSpeed);
        break;
    }
  }

  private void setIntakeSpeed(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  /* Telemetry */
  public RollerState getRollerState() {
    return mState;
  }

}
