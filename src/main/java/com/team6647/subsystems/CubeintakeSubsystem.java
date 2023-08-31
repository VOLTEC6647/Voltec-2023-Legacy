/**
 * Written by Juan Pablo Gutiérrez
 * 28 - 08 - 2023
 */

package com.team6647.subsystems;

import com.andromedalib.motorControllers.SuperSparkMax;
import com.andromedalib.motorControllers.IdleManager.GlobalIdleMode;
import com.team6647.util.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CubeintakeSubsystem extends SubsystemBase {

  private static CubeintakeSubsystem instance;

  private static SuperSparkMax pivotMotor = new SuperSparkMax(IntakeConstants.pivotIntakeID, GlobalIdleMode.brake, false, 50);
  private static SuperSparkMax intakeMotor = new SuperSparkMax(IntakeConstants.intakeMotorID, GlobalIdleMode.brake, false, 50);

  public CubeintakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
