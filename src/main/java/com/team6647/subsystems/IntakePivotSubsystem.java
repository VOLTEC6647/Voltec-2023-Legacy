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
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivotSubsystem extends SubsystemBase {

  private static IntakePivotSubsystem instance;
  private static NetworkTable intakePivotTable;
  private static StringEntry pivotStatePublisher;
  private static DoubleEntry pivotPositionPublisher;
  private static DoubleEntry pivotPIDPublisher;
  private static DoubleEntry pivotSetpointPublisher;

  private static SuperSparkMax pivotMotor = new SuperSparkMax(IntakeConstants.pivotIntakeID, GlobalIdleMode.Coast,
      false, 80, IntakeConstants.pivotPositionConversionFactor, IntakeConstants.pivotZeroOffset, false);

  private static AbsoluteEncoder pivotEncoder;

  private static ProfiledPIDController pivotController = new ProfiledPIDController(IntakeConstants.intakeKp,
      IntakeConstants.intakeKi, IntakeConstants.intakeKd, new TrapezoidProfile.Constraints(140, 140));

  private double setpoint = IntakeConstants.intakeHomedPosition;
  private PivotState mPivotState = PivotState.HOMED;

  double pidVal;

  private IntakePivotSubsystem() {
    pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

    intakePivotTable = NetworkTableInstance.getDefault().getTable("IntakeTable/Pivot");
    pivotStatePublisher = intakePivotTable.getStringTopic("PivotState").getEntry(getPivotState().toString());
    pivotPositionPublisher = intakePivotTable.getDoubleTopic("PivotPosition").getEntry(getPivotPosition());
    pivotPIDPublisher = intakePivotTable.getDoubleTopic("PivotPIDValue").getEntry(getPIDValue());
    pivotSetpointPublisher = intakePivotTable.getDoubleTopic("PivotSetpoint").getEntry(getSetpoint());

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

  /**
   * Calculates the current PID value and applies it to the motor
   */
  private void calculatePID() {
    double pidValue = pivotController.calculate(getPivotPosition(), setpoint);

    pidValue = Functions.clamp(pidValue, -0.6, 0.6);
    pidVal = pidValue;

    double total = pidValue * 12;
    //pivotMotor.set(-0.5);

    pivotMotor.setVoltage(total);
  }

  /**
   * Changes the current pivot state
   * 
   * @param newState New {@link PivotState} for the intake
   */
  public void changePivotState(PivotState newState) {
    if (newState == mPivotState)
      return;

    mPivotState = newState;

    switch (newState) {
      case HOMED:
        changeSetpoint(IntakeConstants.intakeHomedPosition);
        break;
      case EXTENDED:
        changeSetpoint(IntakeConstants.intakeExtendedPosition);
        break;
    }
  }

  /**
   * Security function for chaning the setpoint. Always change the setpoint using
   * this method.
   * 
   * @param newSetpoint New setpoint
   */
  private void changeSetpoint(double newSetpoint) {
    if (newSetpoint < IntakeConstants.minIntakePosition || newSetpoint > IntakeConstants.maxIntakePosition) {
      newSetpoint = Functions.clamp(newSetpoint, IntakeConstants.minIntakePosition, IntakeConstants.maxIntakePosition);
    }

    setpoint = newSetpoint;
  }

  /**
   * Resets the PID Values
   */
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

  /**
   * Gets the current pivot position
   * 
   * @return Current pivot position
   */
  private double getPivotPosition() {
    return pivotEncoder.getPosition();
  }

  /**
   * Gets the current {@link PivotState}
   * 
   * @return Current pivot state
   */
  public PivotState getPivotState() {
    return mPivotState;
  }

  /* Debug Telemetry */

  /**
   * Gets the current calculated PID Value
   * 
   * @return PID Value
   */
  private double getPIDValue() {
    return pidVal;
  }

  /**
   * Gets the current pivot setpoint
   * 
   * @return Pivot setpoint
   */
  private double getSetpoint() {
    return setpoint;
  }

}
