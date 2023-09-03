/**
 * Written by Juan Pablo Gutiérrez
 * 28 - 08 - 2023
 */

package com.team6647.subsystems;

import com.andromedalib.math.Functions;
import com.andromedalib.motorControllers.SuperSparkMax;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.team6647.util.Constants.IntakeConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotCubeSubsystem extends SubsystemBase {

  private static PivotCubeSubsystem instance;

  private static SuperSparkMax pivotMotor = new SuperSparkMax(IntakeConstants.pivotIntakeID, GlobalIdleMode.brake,
      false, 80);

  private static AbsoluteEncoder pivotEncoder;

  private static ProfiledPIDController pivotController = new ProfiledPIDController(IntakeConstants.intakeKp,
      IntakeConstants.intakeKi, IntakeConstants.intakeKd, new TrapezoidProfile.Constraints(15, 15));

  private double setpoint = IntakeConstants.intakeHomedPosition;
  private PivotState mPivotState;

  double pidVal;

  private PivotCubeSubsystem() {
    pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    pivotEncoder.setPositionConversionFactor(IntakeConstants.pivotPositionConversionFactor);
    pivotEncoder.setZeroOffset(IntakeConstants.pivotZeroOffset);
    pivotMotor.burnFlash();

    mPivotState = PivotState.HOMED;

    resetPID();
  }

  public static PivotCubeSubsystem getInstance() {
    if (instance == null) {
      instance = new PivotCubeSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // TODO Move over to shufflebaord vars
    SmartDashboard.putNumber("PID Value", pidVal);
    SmartDashboard.putNumber("Setpoint", setpoint);

    calculatePID();
  }

  public enum PivotState {
    HOMED, EXTENDED,
  }

  private void calculatePID() {
    double pidValue = pivotController.calculate(getPivotPosition(), setpoint);

    pidValue = Functions.clamp(pidValue, -0.2, 0.2);
    pidVal = pidValue;

    double total = pidValue * 12;

    pivotMotor.setVoltage(total);
  }

  public void changePivotState(PivotState newState) {
    if (newState == mPivotState)
      return;

    switch (newState) {
      case HOMED:
        mPivotState = newState;
        changeSetpoint(IntakeConstants.intakeHomedPosition);
        break;
      case EXTENDED:
        mPivotState = newState;
        changeSetpoint(IntakeConstants.intakeExtendedPosition);
        break;
    }
  }

  private void changeSetpoint(double newSetpoint) {
    if (newSetpoint < IntakeConstants.minIntakePosition || newSetpoint > IntakeConstants.maxIntakePosition) {
      newSetpoint = Functions.clamp(newSetpoint, IntakeConstants.minIntakePosition, IntakeConstants.maxIntakePosition);
    }

    setpoint = newSetpoint;
  }

  public void resetPID() {
    pivotController.reset(getPivotPosition());
  }

  /* Telemetry Values */

  public double getPivotPosition() {
    return pivotEncoder.getPosition();
  }

  public PivotState getPivotState() {
    return mPivotState;
  }

}
