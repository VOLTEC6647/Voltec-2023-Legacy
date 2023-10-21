/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647.subsystems;

import com.andromedalib.vision.LimelightCamera;
import com.team6647.util.Constants.VisionConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private static VisionSubsystem instance;

  private LimelightCamera limelightCamera;

  private boolean aimingLimelight;

  /** Creates a new VisionSubsystem. */
  private VisionSubsystem() {
    limelightCamera = new LimelightCamera();
    limelightCamera.setPipeline(VisionConstants.aprilLimePipe);

    setLimeLEDMode(0);
  }

  public static VisionSubsystem getInstance() {
    if (instance == null) {
      instance = new VisionSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
  }

  public double getX() {
    return limelightCamera.getX();
  }

  public double getY() {
    return limelightCamera.getY();
  }

  public double getArea(){
    return limelightCamera.getArea();
  }

  public double getYaw(){
    return -limelightCamera.getBotpose()[5];
  }

  public boolean hasValidTarget() {
    return limelightCamera.hasValidTarget() == 1;
  }

  /**
   * Sets the Limelight LED mode
   */
  public void setLimeLEDMode(int mode) {
    limelightCamera.setLedMode(mode);
  }

  /**
   * Toggles the aim for LimelightCamera
   */
  public void toggleLimelightAim() {
    aimingLimelight = !aimingLimelight;
  }

  /**
   * Gets the status of the LimelightAim
   * 
   * @return LimelightAim status
   */
  public boolean getLimelightAim() {
    return aimingLimelight;
  }

  /**
   * Sets the {@link LimelightCamera} pipeline
   * 
   * @param pipeLine Pipeline to be used
   */
  public void setLimePipe(int pipeLine) {
    limelightCamera.setPipeline(pipeLine);
  }

  /**
   * Get current Limelight Pipe
   * 
   * @return Current Limelight pipeline
   */
  public int getLimePipe() {
    return (int) limelightCamera.getPipeline();
  }

  public LimelightCamera getCamera() {
    return limelightCamera;
  }
}
