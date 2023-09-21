/**
 * Written by Juan Pablo Gutiérrez
 * 03 09 2023
 */

package com.team6647.subsystems;

import com.andromedalib.motorControllers.SuperSparkMax;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.team6647.util.Constants.ConeIntakeConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivotSubsystem extends SubsystemBase {

  private static ArmPivotSubsystem instance;

  public static SuperSparkMax leftMotor = new SuperSparkMax(ConeIntakeConstants.leftPivotMotorID, GlobalIdleMode.brake,
      false, 80);
  public static SuperSparkMax rightMotor = new SuperSparkMax(ConeIntakeConstants.rightPivotMotorID,
      GlobalIdleMode.brake, true, 80);

  private ProfiledPIDController pivotController = new ProfiledPIDController(ConeIntakeConstants.pivotKp,
      ConeIntakeConstants.pivotKi, ConeIntakeConstants.pivotKd, new TrapezoidProfile.Constraints(2, 2));

  private AbsoluteEncoder pivotEncoder;

  private ArmPivotSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static ArmPivotSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmPivotSubsystem();
    }
    return instance;
  }
}
