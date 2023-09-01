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
import edu.wpi.first.util.sendable.Sendable;
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
  private PivotState mPivotState;

  private RollerStates mRollerState;

  double pidVal;
  private SendableChooser<PivotState> pivotStateChooser = new SendableChooser<>();
  private SendableChooser<RollerStates> rollerStateChooser = new SendableChooser<>();

  private CubeintakeSubsystem() {
    pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    pivotEncoder.setZeroOffset(25.8448869);
    pivotEncoder.setPositionConversionFactor(100);

    mPivotState = PivotState.HOMED;

    pivotStateChooser.setDefaultOption("Homed", PivotState.HOMED);
    pivotStateChooser.addOption("Extended", PivotState.EXTENDED);

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
    // TODO Move over to shufflebaord vars
    SmartDashboard.putNumber("PID Value", pidVal);

    changePivotState(pivotStateChooser.getSelected());
    changeRollerState(rollerStateChooser.getSelected());

    calculatePID();
  }

  public enum PivotState {
    HOMED, EXTENDED,
  }

  public enum RollerStates {
    STOPPED, COLLECTING, SPITTING
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

  public void changeRollerState(RollerStates newState) {
    if (newState == mRollerState)
      return;

    switch (newState) {
      case STOPPED:
        mRollerState = newState;
        setIntakeSpeed(0);
      case COLLECTING:
        mRollerState = newState;
        setIntakeSpeed(IntakeConstants.intakeSpeed);
      case SPITTING:
        mRollerState = newState;
        setIntakeSpeed(-IntakeConstants.intakeSpeed);
    }
  }

  private void changeSetpoint(double newSetpoint) {
    if (newSetpoint < IntakeConstants.minIntakePosition || newSetpoint > IntakeConstants.maxIntakePosition) {
      newSetpoint = Functions.clamp(newSetpoint, IntakeConstants.minIntakePosition, IntakeConstants.maxIntakePosition);
    }

    setpoint = newSetpoint;
  }

  private void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  /* Telemetry Values */

  public double getPivotPosition() {
    return pivotEncoder.getPosition();
  }

  public SendableChooser<PivotState> getPivotChooser() {
    return pivotStateChooser;
  }

  public SendableChooser<RollerStates> getRollerChooser(){
    return rollerStateChooser;
  }


  public PivotState getPivotState() {
    return mPivotState;
  }

  public RollerStates getRollerState(){
    return mRollerState;
  }
}
