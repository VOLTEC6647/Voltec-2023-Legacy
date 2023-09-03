/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Constants {

        public static class OperatorConstants {
                public static final int kDriverControllerPort = 0;
                public static final int kDriverControllerPort2 = 1;

                public static final CommandXboxController driverController1 = new CommandXboxController(
                                OperatorConstants.kDriverControllerPort);
                public static final CommandXboxController driverController2 = new CommandXboxController(
                                OperatorConstants.kDriverControllerPort2);

        }

        public static class ShuffleboardConstants {
                private static final String kShuffleboardTabName = "Team 6647";
                public static final ShuffleboardTab kShuffleboardTab = Shuffleboard.getTab(kShuffleboardTabName);
        }

        public static class ElevatorConstants {
                public static final int leftMotorID = 15;
                public static final int rightMotorID = 16;

                public static final double elevatorKp = 0;
                public static final double elevatorKi = 0;
                public static final double elevatorKd = 0;

                public static final double elevatorKs = 0;
                public static final double elevatorKg = 0;
                public static final double elevatorKv = 0;
                public static final double elevatorKa = 0;

                public static final int elevatorSwitchID = 0;

                public static final double minElevatorPosition = 0; // TODO SET
                public static final double maxElevatorPosition = 100; // TODO SET

        }

        public static class IntakeConstants {
                public static final int pivotIntakeID = 13;
                public static final int intakeMotorID = 14;

                public static final double intakeKp = 0.065; //0.05
                public static final double intakeKi = 0;
                public static final double intakeKd = 0;//1.3;

                public static final double minIntakePosition = 8;
                public static final double maxIntakePosition = 90;

                public static final double intakeHomedPosition = 28;
                public static final double intakeExtendedPosition = 68;

                public static final double pivotPositionConversionFactor = 100;
                public static final double pivotZeroOffset = 90.0467753;

                public static final double intakeSpeed = 0.5;
        }
}
