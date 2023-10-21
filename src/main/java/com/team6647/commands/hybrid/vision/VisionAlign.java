/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647.commands.hybrid.vision;

import com.andromedalib.andromedaSwerve.systems.AndromedaSwerve;
import com.team6647.subsystems.VisionSubsystem;
import com.team6647.util.Constants.VisionConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class VisionAlign extends CommandBase {
  private VisionSubsystem visionSubsystem;
  private AndromedaSwerve andromedaSwerve;
  private final PIDController drivePID, strafePID, rotationPID;

  /** Creates a new VisionAlign. */
  public VisionAlign(VisionSubsystem visionSubsystem, AndromedaSwerve andromedaSwerve) {
    this.andromedaSwerve = andromedaSwerve;
    this.visionSubsystem = visionSubsystem;

    this.drivePID = new PIDController(
        VisionConstants.kPdrive,
        VisionConstants.kIdrive,
        VisionConstants.kDdrive);

    this.strafePID = new PIDController(
        VisionConstants.kPstrafe,
        VisionConstants.kIstrafe,
        VisionConstants.kDstrafe);

    this.rotationPID = new PIDController(
        VisionConstants.kProt,
        VisionConstants.kProt,
        VisionConstants.kProt);

    addRequirements(visionSubsystem, andromedaSwerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (visionSubsystem.hasValidTarget()) {
      return;
    }

    var yAdjust = drivePID.calculate(visionSubsystem.getArea());
    var xAdjust = strafePID.calculate(visionSubsystem.getX());
    var rotAdjust = rotationPID.calculate(visionSubsystem.getYaw());

 /*    double kpAim = VisionConstants.kpAim, kpDistance = VisionConstants.kpDistance,
        min_aim_command = VisionConstants.min_aim_command;

    var steeringAdjust = tx > 1 ? (kpAim * -tx - min_aim_command)
        : (tx < -1 ? (kpAim * -tx + min_aim_command) : 0.0);
    var distanceAdjust = kpDistance * ty; */

    Translation2d translation = new Translation2d(xAdjust, yAdjust);

    andromedaSwerve.drive(translation, rotAdjust, false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
