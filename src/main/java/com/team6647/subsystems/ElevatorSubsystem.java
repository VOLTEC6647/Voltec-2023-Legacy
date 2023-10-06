/**
 * Written by Juan Pablo Gutiérrez
 * 28 - 08 - 2023
 */

package com.team6647.subsystems;

import com.andromedalib.math.Functions;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.andromedalib.motorControllers.SuperSparkMax;
import com.team6647.util.Constants.ElevatorConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  private static ElevatorSubsystem instance;
  private static NetworkTable elevatorTable;
  private static StringEntry elevatorStateEntry;
  private static DoubleEntry elevatorPositionEntry;
  private static DoubleEntry elevatorPIDEntry;
  private static DoubleEntry elevatorSetpointEntry;
  private static BooleanEntry elevatorLimitSwitchEntry;
  private static BooleanPublisher elevatorPIDEnabledEntry;

  private static SuperSparkMax leftMotor = new SuperSparkMax(ElevatorConstants.leftMotorID, GlobalIdleMode.Coast, true,
      80);
  private static SuperSparkMax rightMotor = new SuperSparkMax(ElevatorConstants.rightMotorID, GlobalIdleMode.Coast,
      false,
      80);

  private static ProfiledPIDController elevatorController = new ProfiledPIDController(ElevatorConstants.elevatorKp,
      ElevatorConstants.elevatorKi, ElevatorConstants.elevatorKd, new TrapezoidProfile.Constraints(2, 2));

  private static DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.elevatorSwitchID);

  private double setPoint = 0;
  private double pidVal = 0;
  private ElevatorState mState = ElevatorState.HOMED;

  private boolean pidEnabled;

  private ElevatorSubsystem() {
    leftMotor.setSoftLimit(SoftLimitDirection.kForward, ElevatorConstants.minElevatorSoftLimit);
    leftMotor.setSoftLimit(SoftLimitDirection.kReverse, ElevatorConstants.maxElevatorSoftLimit);
    leftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    leftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    rightMotor.setSoftLimit(SoftLimitDirection.kForward, ElevatorConstants.minElevatorSoftLimit);
    rightMotor.setSoftLimit(SoftLimitDirection.kReverse, ElevatorConstants.maxElevatorSoftLimit);
    rightMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    rightMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    elevatorTable = NetworkTableInstance.getDefault().getTable("ElevatorTable");
    elevatorStateEntry = elevatorTable.getStringTopic("ElevatorState").getEntry(getElevatorState().toString());
    elevatorPositionEntry = elevatorTable.getDoubleTopic("ElevatorPosition").getEntry(getElevatorPosition());
    elevatorPIDEntry = elevatorTable.getDoubleTopic("ElevatorPID").getEntry(getPIDValue());
    elevatorSetpointEntry = elevatorTable.getDoubleTopic("ElevatorSetpoint").getEntry(getSetpoint());
    elevatorLimitSwitchEntry = elevatorTable.getBooleanTopic("ElevatorLimitSwitch").getEntry(getLimitState());
    elevatorPIDEnabledEntry = elevatorTable.getBooleanTopic("ElevatorPIDEnabled").getEntry(getPIDEnabled());
  }

  public static ElevatorSubsystem getInstance() {
    if (instance == null) {
      instance = new ElevatorSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
    if (pidEnabled) {
      moveElevator();
    }
    updateNT();
  }

  public enum ElevatorState {
    HOMED, BOTTOM, MID, MAX, HUMAN_PLAYER
  }

  /**
   * Changes the elevator state
   * 
   * @param newState the new state
   */
  public void changeElevatorState(ElevatorState newState) {
    switch (newState) {
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
      case HUMAN_PLAYER:
        changeSetpoint(ElevatorConstants.elevatorHumanPlayerPosition);
        break;
    }
  }

  /**
   * Security method to prevent the elevator from going over its limits
   * 
   * @param newValue the new setpoint
   */
  private void changeSetpoint(double newValue) {
    if (newValue < ElevatorConstants.minElevatorSoftLimit || newValue > ElevatorConstants.maxElevatorSoftLimit) {
      newValue = Functions.clamp(newValue, ElevatorConstants.minElevatorSoftLimit,
          ElevatorConstants.maxElevatorSoftLimit);
    }
    setPoint = newValue;
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
   * Enalbes the PID Controller
   */
  public void enablePID() {
    pidEnabled = true;
  }

  /**
   * Disables the PID controller
   */
  public void disablePID() {
    pidEnabled = false;
  }

  /**
   * Gets the current PID enabled state
   * 
   * @return true if PID is enabled, false if not
   */
  public boolean getPIDEnabled() {
    return pidEnabled;
  }

  /**
   * Manually moves the elevator
   * 
   * @param speed
   */
  public void moveElevator(double speed) {
    if (getPIDEnabled())
      return;

    if (elevatorHomed(speed))
      return;

    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  /**
   * Stops the elevator on touching the limit switch
   * 
   * @param speed
   * @return
   */
  public boolean elevatorHomed(double speed) {
    if (speed > 0 && getLimitState()) {
      return false;
    }
    return getLimitState();
  }

  /* Telemetry */

  /**
   * Updates al NetworkTable values
   */
  private void updateNT() {
    elevatorStateEntry.set(getElevatorState().toString());
    elevatorPositionEntry.set(getElevatorPosition());
    elevatorPIDEntry.set(getPIDValue());
    elevatorSetpointEntry.set(getSetpoint());
    elevatorLimitSwitchEntry.set(getLimitState());
    elevatorPIDEnabledEntry.set(getPIDEnabled());
  }

  /**
   * Gets the current elevator position in motor counts
   * 
   * @return Elevator position
   */
  public double getElevatorPosition() {
    return leftMotor.getPosition();
  }

  /**
   * Resets the elevator position
   */
  public void resetElevatorPosition() {
    leftMotor.resetEncoder();
  }

  /**
   * Gets the current state of the limit switch
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
  private ElevatorState getElevatorState() {
    return mState;
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
