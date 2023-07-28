package frc.robot;

import com.andromedalib.andromedaSwerve.commands.SwerveDriveCommand;
import com.andromedalib.andromedaSwerve.systems.AndromedaSwerve;
import com.andromedalib.robot.SuperRobotContainer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.util.Constants.OperatorConstants;

public class RobotContainer extends SuperRobotContainer {

  private AndromedaSwerve andromedaSwerve;

  private static RobotContainer instance;

  private RobotContainer() {
  }

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }

    return instance;
  }

  @Override
  public void initSubsystems() {
    andromedaSwerve = AndromedaSwerve.getInstance();
  }

  @Override
  public void configureBindings() {

    andromedaSwerve.setDefaultCommand(
        new SwerveDriveCommand(
            andromedaSwerve,
            () -> -OperatorConstants.driverController1.getLeftY(),
            () -> OperatorConstants.driverController1.getLeftX(),
            () -> -OperatorConstants.driverController1.getRightX(),
            () -> OperatorConstants.driverController1.leftStick().getAsBoolean()));
  }

  @Override
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
