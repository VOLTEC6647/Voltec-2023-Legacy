/**
 * Written by Juan Pablo Gutiérrez
 * 28 - 08 - 2023
 */

package com.team6647.subsystems;

import com.andromedalib.math.Functions;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.andromedalib.motorControllers.SuperSparkMax;
import com.team6647.util.Constants.ElevatorConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  private static ElevatorSubsystem instance;
  private static NetworkTable elevatorTable;
  private static StringEntry elevatorPositionStateEntry;
  private static StringEntry elevatorPIDStateEntry;
  private static DoubleEntry elevatorPositionEntry;
  private static DoubleEntry elevatorEncoderPositionEntry;
  private static DoubleEntry elevatorPIDEntry;
  private static DoubleEntry elevatorSetpointEntry;
  private static BooleanEntry elevatorLimitSwitchEntry;

  private static SuperSparkMax leftMotor = new SuperSparkMax(ElevatorConstants.leftMotorID, GlobalIdleMode.Coast, false,
      80, ElevatorConstants.elevatorEncoderPositionConversionFactor, ElevatorConstants.elevatorEncoderZeroOffset,
      ElevatorConstants.elevatorEncoderInverted);
  private static SuperSparkMax rightMotor = new SuperSparkMax(ElevatorConstants.rightMotorID, GlobalIdleMode.Coast,
      true,
      80);

  private static AbsoluteEncoder elevatorEncoder;

  private static ProfiledPIDController elevatorController = new ProfiledPIDController(ElevatorConstants.elevatorKp,
      ElevatorConstants.elevatorKi, ElevatorConstants.elevatorKd, new TrapezoidProfile.Constraints(200, 150));

  private static DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.elevatorSwitchID);

  private double setPoint = ElevatorConstants.minElevatorSoftLimit;
  private ElevatorPositionState mPositionState = ElevatorPositionState.HOMED;
  private ElevatorState mElevatorState = ElevatorState.PID;

  private double pidVal = 0;

  private ElevatorSubsystem() {
    elevatorEncoder = leftMotor.getAbsoluteEncoder(Type.kDutyCycle);

    elevatorController.setTolerance(5);

    elevatorTable = NetworkTableInstance.getDefault().getTable("ElevatorTable");
    elevatorPositionStateEntry = elevatorTable.getStringTopic("ElevatorPositionState")
        .getEntry(getElevatorPositionState().toString());
    elevatorPIDStateEntry = elevatorTable.getStringTopic("ElevatorPIDState").getEntry(getElevatorState().toString());
    elevatorPositionEntry = elevatorTable.getDoubleTopic("ElevatorPosition").getEntry(getElevatorPosition());
    elevatorEncoderPositionEntry = elevatorTable.getDoubleTopic("ElevatorEncoderPosition")
        .getEntry(getElevatorEncoderPosition());
    elevatorPIDEntry = elevatorTable.getDoubleTopic("ElevatorPID").getEntry(getPIDValue());
    elevatorSetpointEntry = elevatorTable.getDoubleTopic("ElevatorSetpoint").getEntry(getSetpoint());
    elevatorLimitSwitchEntry = elevatorTable.getBooleanTopic("ElevatorLimitSwitch").getEntry(getLimitState());
  }

  public static ElevatorSubsystem getInstance() {
    if (instance == null) {
      instance = new ElevatorSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
    if (getElevatorState() == ElevatorState.PID) {
      moveElevator();
    }
    updateNT();
  }

  public enum ElevatorPositionState {
    FlOOR, HOMED, BOTTOM, MID, MAX, HUMAN_PLAYER, MID_CONE, MAX_CONE
  }

  public enum ElevatorState {
    PID, MANUAL
  }

  /**
   * Changes the elevator position state
   * 
   * @param newState New state
   */
  public void changeElevatorPositionState(ElevatorPositionState newState) {
    if (newState == mPositionState)
      return;

    mPositionState = newState;

    switch (newState) {
      case FlOOR:
        changeSetpoint(ElevatorConstants.elevatorFloorPosition);
        break;
      case HOMED:
        changeSetpoint(ElevatorConstants.elevatorHomedPosition);
        break;
      case BOTTOM:
        changeSetpoint(ElevatorConstants.elevatorBottomPosition);
        break;
      case MID:
        changeSetpoint(ElevatorConstants.elevatorMiddlePosition);
        break;
      case MAX:
        changeSetpoint(ElevatorConstants.elevatorTopPosition);
        break;
      case MID_CONE:
        changeSetpoint(ElevatorConstants.elevatorMiddleConePosition);
        break;
      case MAX_CONE:
        changeSetpoint(ElevatorConstants.elevatorTopConePosition);
        break;
      case HUMAN_PLAYER:
        changeSetpoint(ElevatorConstants.elevatorHumanPlayerPosition);
        break;
    }
  }

  /**
   * Changes the elevator control state
   * 
   * @param newState New State
   */
  public void changeElevatorState(ElevatorState newState) {
    if (newState == mElevatorState)
      return;

    mElevatorState = newState;
  }

  /**
   * Security method to prevent the elevator from going over its limits
   * 
   * @param newValue the new setpoint
   */
  private void changeSetpoint(double newValue) {
    if (newValue < ElevatorConstants.minElevatorSoftLimit || newValue > ElevatorConstants.elevatorTopConePosition) {
      newValue = Functions.clamp(newValue, ElevatorConstants.minElevatorSoftLimit,
          ElevatorConstants.maxElevatorSoftLimit);
    }
    setPoint = newValue;
  }

  public void manualMovement(double newVal) {
    changeSetpoint(setPoint += newVal);
  }

  /**
   * Moves the elevator to the setpoint
   * using the PID controller
   */
  private void moveElevator() {
    double pidValue = elevatorController.calculate(getElevatorPosition(), setPoint);

    pidValue = Functions.clamp(pidValue, -0.4, 0.4);
    pidVal = pidValue;

    double total = pidValue * 12;

    leftMotor.setVoltage(total);
    rightMotor.setVoltage(total);
  }

  /**
   * Manually moves the elevator. Will only move if the {@link ElevatorState} is
   * set to {@link ElevatorState#MANUAL}
   * 
   * @param speed Speed of the elevator
   */
  public void manualMoveElevator(double speed) {
    if (mElevatorState == ElevatorState.PID)
      return;

    if (elevatorHomed(speed))
      return;

    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  /**
   * Stops the elevator on touching the limit switch
   * 
   * @param speed Speed of the elevator
   * @return True if the elevator is homed
   */
  public boolean elevatorHomed(double speed) {
    if (speed > 0 && getLimitState()) {
      return false;
    }
    return getLimitState();
  }

  public void resetEncoderToAbsolute() {
    leftMotor.setPosition(getElevatorEncoderPosition());
  }

  public boolean isInTolerance() {
    return elevatorController.atGoal();
  }

  /* Telemetry */

  /**
   * Updates al NetworkTable values
   */
  private void updateNT() {
    elevatorPositionStateEntry.set(getElevatorPositionState().toString());
    elevatorPositionEntry.set(getElevatorPosition());
    elevatorEncoderPositionEntry.set(getElevatorEncoderPosition());
    elevatorPIDStateEntry.set(getElevatorState().toString());
    elevatorPIDEntry.set(getPIDValue());
    elevatorSetpointEntry.set(getSetpoint());
    elevatorLimitSwitchEntry.set(getLimitState());
  }

  /**
   * Gets the current elevator position in motor counts
   * 
   * @return Elevator position
   */
  public double getElevatorPosition() {
    return leftMotor.getPosition();
  }

  public double getElevatorEncoderPosition() {
    return elevatorEncoder.getPosition();
  }

  /**
   * Resets the elevator position
   */
  public void resetElevatorPosition() {
    leftMotor.resetEncoder();
  }

  /**
   * Gets the current state of the limit switch
   * abejita was here :)
   * 
   * @return Current limit switch state
   */
  public boolean getLimitState() {
    return limitSwitch.get();
  }

  /**
   * Gets the current state of the elevator
   * 
   * @return Current elevator state
   */
  private ElevatorPositionState getElevatorPositionState() {
    return mPositionState;
  }

  /**
   * Gets the current PID enabled state
   * 
   * @return true if PID is enabled, false if not
   */
  public ElevatorState getElevatorState() {
    return mElevatorState;
  }

  /**
   * Gets the current PID value
   * 
   * @return Current PID value
   */
  private double getPIDValue() {
    return pidVal;
  }

  /**
   * Gets the current setpoint
   * 
   * @return Current setpoint
   */
  private double getSetpoint() {
    return setPoint;
  }
}
