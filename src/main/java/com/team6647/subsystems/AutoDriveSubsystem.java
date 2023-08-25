/**
 * Written by Juan Pablo Gutiérrez
 * 17 - 08 - 2023
 */
package com.team6647.subsystems;

import com.andromedalib.andromedaSwerve.systems.AndromedaSwerve;
import com.andromedalib.andromedaSwerve.utils.SwerveConstants;
import com.andromedalib.vision.LimelightHelpers;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoDriveSubsystem extends SubsystemBase {

  private static AutoDriveSubsystem instance;

  AndromedaSwerve swerve;

  SwerveDrivePoseEstimator poseEstimator;

  Field2d field;

  Alliance alliance;

  /** Creates a new AutoDriveSubsystem. */
  private AutoDriveSubsystem(AndromedaSwerve swerve) {
    this.swerve = swerve;

    field = new Field2d();

    this.poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.swerveKinematics, swerve.getAngle(),
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
    poseEstimator.update(swerve.getAngle(), swerve.getPositions());
    field.setRobotPose(getPose());
    SmartDashboard.putData(field);

    LimelightHelpers.Results result = LimelightHelpers.getLatestResults("limelight").targetingResults;

    if (!(result.botpose[0] == 0 && result.botpose[1] == 0) && LimelightHelpers.getTA("limelight") < 30) {
      if (alliance == Alliance.Blue) {
        poseEstimator.addVisionMeasurement(
            LimelightHelpers.toPose2D(result.botpose_wpiblue),
            Timer.getFPGATimestamp() - (result.latency_capture / 1000.0) - (result.latency_pipeline / 1000.0));
      } else if (alliance == Alliance.Red) {
        poseEstimator.addVisionMeasurement(
            LimelightHelpers.toPose2D(result.botpose_wpired),
            Timer.getFPGATimestamp() - (result.latency_capture / 1000.0) - (result.latency_pipeline / 1000.0));
      }
    }
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
    poseEstimator.resetPosition(swerve.getAngle(), swerve.getPositions(), pose);
  }

  /**
   * Created a new command that follows a defined PathPlanner Trajectory
   * 
   * @param trajectory  Trajectory to follow
   * @param isFirstPath If true, it will reset the odometry
   * @return FollowTrajectory Command to follow
   */
  public Command followTrajectoryCommand(PathPlannerTrajectory trajectory, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          if (isFirstPath) {
            this.resetOdometry(trajectory.getInitialHolonomicPose());
          }
        }),

        new PPSwerveControllerCommand(
            trajectory,
            this::getPose,
            SwerveConstants.swerveKinematics,
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0),
            new PIDController(1.75, 0, 0),
            swerve::setModuleStates,
            true,
            swerve));

  }
}
