/**
 * Written by Juan Pablo Gutiérrez
 * 28 - 08 - 2023
 */

package com.team6647.subsystems;

import com.andromedalib.motorControllers.SuperTalonFX;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team6647.subsystems.IntakeSubsystem.RollerState;
import com.team6647.util.Constants.ArmIntakeConstants;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmIntakeSubsytem extends SubsystemBase {
  private static ArmIntakeSubsytem instance;
  private static NetworkTable armIntakeTable;
  private static StringEntry ArmIntakeStateEntry;
  private static BooleanEntry beamBrakeEntry;

  private SuperTalonFX intakeMotor = new SuperTalonFX(ArmIntakeConstants.intakeMotorID, GlobalIdleMode.Coast, true);

  private RollerState mState = RollerState.STOPPED;
  private DigitalInput beamBrake = new DigitalInput(ArmIntakeConstants.beamBrakePort);

  /** Creates a new ConeIntakeSubsystem. */
  private ArmIntakeSubsytem() {

    armIntakeTable = NetworkTableInstance.getDefault().getTable("ArmTable/Intake");
    ArmIntakeStateEntry = armIntakeTable.getStringTopic("ArmIntakeState").getEntry(getRollerState().toString());
    beamBrakeEntry = armIntakeTable.getBooleanTopic("BeamBrake").getEntry(getBeamBrake());
  }

  public static ArmIntakeSubsytem getInstance() {
    if (instance == null) {
      instance = new ArmIntakeSubsytem();
    }
    return instance;
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
        setIntakeSpeed(ArmIntakeConstants.intakeSpeed);
        break;
      case SPITTING:
        mState = RollerState.SPITTING;
        setIntakeSpeed(-ArmIntakeConstants.intakeSpeed);
        break;
    }
  }

  private void setIntakeSpeed(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  /* Telemetry */
  private void updateNT() {
    ArmIntakeStateEntry.set(getRollerState().toString());
    beamBrakeEntry.set(getBeamBrake());
  }

  public RollerState getRollerState() {
    return mState;
  }

  public boolean getBeamBrake() {
    return beamBrake.get();
  }

}
