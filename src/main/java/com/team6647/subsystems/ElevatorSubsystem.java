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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  private static ElevatorSubsystem instance;

  private static SuperSparkMax leftMotor = new SuperSparkMax(ElevatorConstants.leftMotorID, GlobalIdleMode.Coast, true,
      80);
  private static SuperSparkMax rightMotor = new SuperSparkMax(ElevatorConstants.rightMotorID, GlobalIdleMode.Coast,
      false,
      80);

  private static ProfiledPIDController elevatorController = new ProfiledPIDController(ElevatorConstants.elevatorKp,
      ElevatorConstants.elevatorKi, ElevatorConstants.elevatorKd, new TrapezoidProfile.Constraints(2, 2));

  private static DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.elevatorSwitchID);

  private double setPoint = 0;
  private ElevatorState mState;

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

    mState = ElevatorState.STATIC;
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
  }

  public enum ElevatorState {
    STATIC, BOTTOM, MID, MAX,
  }

  public void changeElevatorState(ElevatorState newState) {
    switch (newState) {
      case STATIC:
        changeSetpoint(setPoint);
        break;
      case BOTTOM:
        changeSetpoint(setPoint);
        break;
      case MID:
        changeSetpoint(setPoint);
        break;
      case MAX:
        changeSetpoint(10);
        break;
    }
  }

  private void changeSetpoint(double newValue) {
    if (newValue < ElevatorConstants.minElevatorSoftLimit || newValue > ElevatorConstants.maxElevatorSoftLimit) {
      newValue = Functions.clamp(newValue, ElevatorConstants.minElevatorSoftLimit,
          ElevatorConstants.maxElevatorSoftLimit);
    }
    setPoint = newValue;
  }

  private void moveElevator() {
    double pidValue = elevatorController.calculate(getElevatorPosition(), setPoint);

    pidValue = Functions.clamp(pidValue, -0.4, 0.4);

    SmartDashboard.putNumber("PID VALUE ELVATOR", pidValue);

    double total = pidValue * 12;

    leftMotor.setVoltage(total);
    rightMotor.setVoltage(total);
  }

  public void enablePID() {
    pidEnabled = true;
  }

  public void disablePID() {
    pidEnabled = false;
  }

  public void moveElevator(double speed) {
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  /**
   * Gets the current limit switch stat
   * 
   * @return Limit switch state
   */
  public boolean getLimitSwitch() {
    return limitSwitch.get();
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
  public ElevatorState getElevatorState() {
    return mState;
  }
}
