/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
