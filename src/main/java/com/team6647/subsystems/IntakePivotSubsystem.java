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
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivotSubsystem extends SubsystemBase {

  private static IntakePivotSubsystem instance;
  private static NetworkTable intakePivotTable;
  private static StringPublisher pivotStatePublisher;
  private static DoublePublisher pivotPositionPublisher;
  private static DoublePublisher pivotPIDPublisher;
  private static DoublePublisher pivotSetpointPublisher;

  private static SuperSparkMax pivotMotor = new SuperSparkMax(IntakeConstants.pivotIntakeID, GlobalIdleMode.brake,
      false, 80, IntakeConstants.pivotPositionConversionFactor, IntakeConstants.pivotZeroOffset, true);

  private static AbsoluteEncoder pivotEncoder;

  private static ProfiledPIDController pivotController = new ProfiledPIDController(IntakeConstants.intakeKp,
      IntakeConstants.intakeKi, IntakeConstants.intakeKd, new TrapezoidProfile.Constraints(60, 55));

  private double setpoint = IntakeConstants.intakeHomedPosition;
  private PivotState mPivotState;

  double pidVal;

  private IntakePivotSubsystem() {
    pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

    mPivotState = PivotState.HOMED;

    intakePivotTable  = NetworkTableInstance.getDefault().getTable("IntakePivotTable");
    pivotStatePublisher = intakePivotTable.getStringTopic("PivotState").publish();
    pivotPositionPublisher = intakePivotTable.getDoubleTopic("PivotPosition").publish();
    pivotPIDPublisher = intakePivotTable.getDoubleTopic("PivotPIDValue").publish();
    pivotSetpointPublisher = intakePivotTable.getDoubleTopic("PivotSetpoint").publish();

    resetPID();
  }

  public static IntakePivotSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakePivotSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
    calculatePID();
    updateNT();
  }

  public enum PivotState {
    HOMED, EXTENDED,
  }

  private void calculatePID() {
    double pidValue = pivotController.calculate(getPivotPosition(), setpoint);

    pidValue = Functions.clamp(pidValue, -0.6, 0.6);
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

  private void updateNT() {
    pivotStatePublisher.set(mPivotState.toString());
    pivotPositionPublisher.set(getPivotPosition());
    pivotPIDPublisher.set(getPIDValue());
    pivotSetpointPublisher.set(getSetpoint());
  }

  public double getPivotPosition() {
    return pivotEncoder.getPosition();
  }

  public PivotState getPivotState() {
    return mPivotState;
  }

  /* Debug Telemetry */
  public double getPIDValue() {
    return pidVal;
  }

  public double getSetpoint() {
    return setpoint;
  }

}
