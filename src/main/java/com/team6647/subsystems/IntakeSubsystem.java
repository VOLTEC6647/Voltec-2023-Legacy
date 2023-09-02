/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647.subsystems;

import com.andromedalib.motorControllers.SuperSparkMax;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.team6647.util.Constants.IntakeConstants;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private static IntakeSubsystem instance;

  private static SuperSparkMax intakeMotor = new SuperSparkMax(IntakeConstants.intakeMotorID, GlobalIdleMode.Coast,
      false, 50);

  private RollerStates mRollerState;

  private SendableChooser<RollerStates> rollerStateChooser = new SendableChooser<>();

  /** Creates a new PivotCubeIntakeSubsystem. */
  public IntakeSubsystem() {
    mRollerState = RollerStates.STOPPED;

    rollerStateChooser.setDefaultOption("Stopped", RollerStates.STOPPED);
    rollerStateChooser.addOption("Collecting", RollerStates.COLLECTING);
    rollerStateChooser.addOption("Spitting", RollerStates.SPITTING);
  }

  public static IntakeSubsystem getInstance() {
    if (instance == null)
      instance = new IntakeSubsystem();
    return instance;
  }

  @Override
  public void periodic() {
    changeRollerState(rollerStateChooser.getSelected());
  }

  public enum RollerStates {
    STOPPED, COLLECTING, SPITTING
  }

  public void changeRollerState(RollerStates newState) {
    if (newState == mRollerState)
      return;
      
    switch (newState) {
      case STOPPED:
        mRollerState = newState;
        setIntakeSpeed(0);
      case COLLECTING:
        mRollerState = newState;
        setIntakeSpeed(IntakeConstants.intakeSpeed);
      case SPITTING:
        mRollerState = newState;
        setIntakeSpeed(-IntakeConstants.intakeSpeed);
    }
  }

  private void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public SendableChooser<RollerStates> getRollerChooser() {
    return rollerStateChooser;
  }

  public RollerStates getRollerState() {
    return mRollerState;
  }

}
