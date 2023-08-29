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
import com.team6647.util.Constants.ElevatorConstants;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  private static ElevatorSubsystem instance;

  private static SuperSparkMax leftMotor = new SuperSparkMax(ElevatorConstants.leftMotorID, GlobalIdleMode.brake, false,
      50);
  private static SuperSparkMax rightMotor = new SuperSparkMax(ElevatorConstants.rightMotorID, GlobalIdleMode.brake,
      true,
      50);

  private static AbsoluteEncoder elevatorEncoder;

  private static ProfiledPIDController elevatorController = new ProfiledPIDController(ElevatorConstants.elevatorKp,
      ElevatorConstants.elevatorKi, ElevatorConstants.elevatorKd, new TrapezoidProfile.Constraints(2, 2));
  private static ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.elevatorKs,
      ElevatorConstants.elevatorKg, ElevatorConstants.elevatorKv, ElevatorConstants.elevatorKa);

  private static DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.elevatorSwitchID);

  private double setPoint = 0;
  private elevatorStates mState;

  private ElevatorSubsystem() {
    rightMotor.follow(leftMotor);

    elevatorEncoder = leftMotor.getAbsoluteEncoder(Type.kDutyCycle);

    mState = elevatorStates.STATIC;

  }

  public static ElevatorSubsystem getInstance() {
    if (instance == null) {
      instance = new ElevatorSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
  }

  public enum elevatorStates {
    STATIC, MOVING,
  }

  public void changeSetpoint(double newValue) {
    if (newValue < ElevatorConstants.minElevatorPosition || newValue > ElevatorConstants.maxElevatorPosition) {
      newValue = Functions.clamp(newValue, ElevatorConstants.minElevatorPosition,
          ElevatorConstants.maxElevatorPosition);
    }
    setPoint = newValue;
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
    return elevatorEncoder.getPosition();
  }
}
