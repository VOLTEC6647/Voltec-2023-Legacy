/**
 * Written by Juan Pablo Gutiérrez
 */
package frc.robot;

import com.andromedalib.robot.SuperRobot;

import frc.util.shuffleboard.TelemetryManager;

public class Robot extends SuperRobot {
  private RobotContainer container;

  @Override
  public void robotInit() {
    container = RobotContainer.getInstance();

    super.setRobotContainer(container, TelemetryManager.getInstance(), false);

    super.robotInit();
  }
}
