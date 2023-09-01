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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CubeintakeSubsystem extends SubsystemBase {

  private static CubeintakeSubsystem instance;

  private static SuperSparkMax pivotMotor = new SuperSparkMax(IntakeConstants.pivotIntakeID, GlobalIdleMode.brake,
      false, 50);
  private static SuperSparkMax intakeMotor = new SuperSparkMax(IntakeConstants.intakeMotorID, GlobalIdleMode.brake,
      false, 50);

  private static AbsoluteEncoder pivotEncoder;

  private static ProfiledPIDController pivotController = new ProfiledPIDController(IntakeConstants.intakeKp,
      IntakeConstants.intakeKi, IntakeConstants.intakeKd, new TrapezoidProfile.Constraints(2, 2));

  private double setpoint = 2;
  private PivotState mState;

  double pidVal;
  SendableChooser<PivotState> stateChooser = new SendableChooser<>();

  private CubeintakeSubsystem() {
    pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

    mState = PivotState.HOMED;

    stateChooser.setDefaultOption("Homed", PivotState.HOMED);
    stateChooser.addOption("Extended", PivotState.EXTENDED);

    pivotController.reset(getPivotPosition());
  }

  public static CubeintakeSubsystem getInstance() {
    if (instance == null) {
      instance = new CubeintakeSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("PID Value", pidVal);

    SmartDashboard.putData("Pivot", stateChooser);

    SmartDashboard.putNumber("Position", getPivotPosition());

    changeState(stateChooser.getSelected());

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

  public void changeState(PivotState newState) {
    switch (newState) {
      case HOMED:
        mState = newState;
        changeSetpoint(2);
        break;
      case EXTENDED:
        mState = newState;
        changeSetpoint(42);
        break;
    }
  }

  private void changeSetpoint(double newSetpoint) {
    if (newSetpoint < IntakeConstants.minIntakePosition || newSetpoint > IntakeConstants.maxIntakePosition) {
      newSetpoint = Functions.clamp(newSetpoint, IntakeConstants.minIntakePosition, IntakeConstants.maxIntakePosition);
    }

    setpoint = newSetpoint;
  }

  public double getPivotPosition() {
    return pivotEncoder.getPosition();
  }
}
