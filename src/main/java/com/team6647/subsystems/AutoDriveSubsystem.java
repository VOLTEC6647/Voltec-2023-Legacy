/**
 * Written by Juan Pablo Gutiérrez
 * 17 - 08 - 2023
 */
package com.team6647.subsystems;

import java.util.List;

import com.andromedalib.andromedaSwerve.systems.AndromedaSwerve;
import com.andromedalib.andromedaSwerve.utils.SwerveConstants;
import com.andromedalib.sensors.SuperNavx;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.team6647.util.Constants.DriveConstants;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoDriveSubsystem extends SubsystemBase {

  private static AutoDriveSubsystem instance;

  AndromedaSwerve swerve;

  SwerveDrivePoseEstimator poseEstimator;

  Field2d field;

  Alliance alliance;

  SuperNavx navx = SuperNavx.getInstance();

  /** Creates a new AutoDriveSubsystem. */
  private AutoDriveSubsystem(AndromedaSwerve swerve) {
    this.swerve = swerve;

    field = new Field2d();

    this.poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.swerveKinematics, swerve.getSwerveAngle(),
        swerve.getPositions(), new Pose2d());

    resetOdometry(new Pose2d());

    this.alliance = DriverStation.getAlliance();

  }

  public static AutoDriveSubsystem getInstance(AndromedaSwerve swerve) {
    if (instance == null) {
      instance = new AutoDriveSubsystem(swerve);
    }
    return instance;
  }

  @Override
  public void periodic() {
    poseEstimator.update(swerve.getSwerveAngle(), swerve.getPositions());
    computeVisionMeasurements();

    field.setRobotPose(getPose());
    SmartDashboard.putData(field);

    SmartDashboard.putNumber("Pitch", getNavxPitch());

  }

  /**
   * Gets current Navx Roll
   * 
   * @return Navx Roll
   */
  public double getNavxRoll() {
    return navx.getRoll();
  }

  public double getNavxPitch() {
    return navx.getPitch();
  }

  /**
   * Computes Limelight MegaBotBose data and adds it into the
   * {@link SwerveDrivePoseEstimator}
   */
  public void computeVisionMeasurements() {
    /*
     * LimelightHelpers.Results result =
     * LimelightHelpers.getLatestResults("limelight").targetingResults;
     * 
     * if (!(result.botpose[0] == 0 && result.botpose[1] == 0) &&
     * LimelightHelpers.getTA("limelight") < 30) {
     * if (alliance == Alliance.Blue) {
     * poseEstimator.addVisionMeasurement(
     * LimelightHelpers.toPose2D(result.botpose_wpiblue),
     * Timer.getFPGATimestamp() - (result.latency_capture / 1000.0) -
     * (result.latency_pipeline / 1000.0));
     * } else if (alliance == Alliance.Red) {
     * poseEstimator.addVisionMeasurement(
     * LimelightHelpers.toPose2D(result.botpose_wpired),
     * Timer.getFPGATimestamp() - (result.latency_capture / 1000.0) -
     * (result.latency_pipeline / 1000.0));
     * }
     * }
     */
  }

  /**
   * Gets the current robot pose calculated by the odometry
   * 
   * @return Current pose
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to a defined Pose2D
   * 
   * @param pose New Pose2D
   */
  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(swerve.getSwerveAngle(), swerve.getPositions(), pose);
  }

  public Command getFullAuto(String pathName) {

    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(1.5, 2),
        new PathConstraints(3, 2));

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        this::getPose,
        this::resetOdometry,
        SwerveConstants.swerveKinematics,
        new PIDConstants(1.7, 0.0, 0.0),
        new PIDConstants(2, 0.0, 0.0),
        swerve::setModuleStates,
        DriveConstants.eventMap,
        true,
        swerve);

    Command fullAuto = autoBuilder.fullAuto(pathGroup);

    return fullAuto;
  }
}
