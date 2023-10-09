/**
 * Written by Juan Pablo Gutiérrez
 * 03 09 2023
 */

package com.team6647.subsystems;

import com.andromedalib.math.Functions;
import com.andromedalib.motorControllers.SuperSparkMax;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.team6647.util.Constants.ArmIntakeConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivotSubsystem extends SubsystemBase {

  private static ArmPivotSubsystem instance;
  private static NetworkTable armPivotTable;
  private static StringEntry armStateEntry;
  private static DoubleEntry armPositionDoubleEntry;
  private static DoubleEntry armPIDDoubleEntry;
  private static DoubleEntry armSetpointDoubleEntry;

  public static SuperSparkMax leftMotor = new SuperSparkMax(ArmIntakeConstants.leftPivotMotorID, GlobalIdleMode.brake,
      false, 80);
  public static SuperSparkMax rightMotor = new SuperSparkMax(ArmIntakeConstants.rightPivotMotorID,
      GlobalIdleMode.brake, false, 80);

  private ProfiledPIDController armController = new ProfiledPIDController(ArmIntakeConstants.pivotKp,
      ArmIntakeConstants.pivotKi, ArmIntakeConstants.pivotKd, new TrapezoidProfile.Constraints(2, 2));

  private AbsoluteEncoder armEncoder;


  private double setpoint = ArmIntakeConstants.intakeHomedPosition;
  private ArmPivotState mArmState = ArmPivotState.HOMED;

  private double pidVal = 0;

  private ArmPivotSubsystem() {
    armEncoder = leftMotor.getAbsoluteEncoder(Type.kDutyCycle);
    rightMotor.follow(leftMotor);

    armPivotTable = NetworkTableInstance.getDefault().getTable("ArmTable/Pivot");
    armStateEntry = armPivotTable.getStringTopic("ArmPivotState").getEntry(getArmState().toString());
    armPositionDoubleEntry = armPivotTable.getDoubleTopic("ArmPosition").getEntry(getArmPosition());
    armPIDDoubleEntry = armPivotTable.getDoubleTopic("ArmPID").getEntry(getPIDVal());
    armSetpointDoubleEntry = armPivotTable.getDoubleTopic("ArmSetpoint").getEntry(getSetpoint());

    resetPID();
  }

  public static ArmPivotSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmPivotSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
    calculatePID();
    updateNT();
  }

  public enum ArmPivotState {
    HOMED, FLOOR, PLACING, SCORING, INDEXING, HUMAN_PLAYER
  }

  private void calculatePID() {
    double pidValue = armController.calculate(getArmPosition(), setpoint);

    pidValue = Functions.clamp(pidValue, -0.2, 0.2);
    pidVal = pidValue;

    double total = pidValue * 12;

    leftMotor.setVoltage(total);
  }

  public void changeArmState(ArmPivotState newState) {
    if (newState == mArmState)
      return;

    mArmState = newState;

    switch (newState) {
      case HOMED:
        changeSetpoint(ArmIntakeConstants.intakeHomedPosition);
        break;
      case FLOOR:
        changeSetpoint(ArmIntakeConstants.intakeFloorPosition);
        break;
      case PLACING:
        changeSetpoint(ArmIntakeConstants.intakePlacingPositon);
        break;
      case SCORING:
        changeSetpoint(ArmIntakeConstants.intakeScoringPositon);
        break;
      case INDEXING:
        changeSetpoint(ArmIntakeConstants.intakeIndexingPosition);
        break;
      case HUMAN_PLAYER:
        changeSetpoint(ArmIntakeConstants.intakeHumanPlayerPosition);
        break;
    }
  }

  /**
   * Security function for chaning the setpoint. Always change the setpoint using
   * this method.
   * 
   * @param newSetpoint New setpoint
   */
  public void changeSetpoint(double newSetpoint) {
    if (newSetpoint < ArmIntakeConstants.intakeFloorPosition || newSetpoint > ArmIntakeConstants.intakeScoringPositon) {
      newSetpoint = Functions.clamp(newSetpoint, ArmIntakeConstants.intakeFloorPosition,
          ArmIntakeConstants.intakeScoringPositon);
    }

    setpoint = newSetpoint;
  }

  private void resetPID() {
    armController.reset(getArmPosition());
  }

  /* Telemetry */
  private void updateNT() {
    armStateEntry.set(getArmState().toString());
    armPositionDoubleEntry.set(getArmPosition());
    armPIDDoubleEntry.set(getPIDVal());
    armSetpointDoubleEntry.set(getSetpoint());
  }

  private double getArmPosition() {
    return armEncoder.getPosition();
  }

  private ArmPivotState getArmState() {
    return mArmState;
  }

  public double getPIDVal() {
    return pidVal;
  }

  public double getSetpoint() {
    return setpoint;
  }
}
