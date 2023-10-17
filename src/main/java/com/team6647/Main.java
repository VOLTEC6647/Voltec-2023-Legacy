/**
 * Created by Juan Pablo Gutiérrez
 * 
 * Voltec 2023 Legacy
 * 
 * "If you're going to leave a mark, make it a legacy."
 */
package com.team6647;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {
  }

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}

