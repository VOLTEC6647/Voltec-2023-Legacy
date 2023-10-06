/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647.util;

import java.util.HashMap;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;


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

        public static class DriveConstants {
                public static final double balanceGoal = 0;
                public static final double balanceKp = 0.65;
                public static final double balanceTolerance = 11;

                public static HashMap<String, Command> eventMap = new HashMap<>();

        }

        public static class ElevatorConstants {
                public static final int leftMotorID = 15;
                public static final int rightMotorID = 16;

                public static final double elevatorKp = 0.3;
                public static final double elevatorKi = 0;
                public static final double elevatorKd = 0;

                public static final int elevatorSwitchID = 1;
                public static final int beamBrakePort = 2;

                public static final float minElevatorSoftLimit = 0; // TODO SET
                public static final float maxElevatorSoftLimit = 100; // TODO SET

                public static final double elevatorHomedPosition = 0; // TODO SET
                public static final double elevatorBottomPosition = 0; // TODO SET
                public static final double elevatorMiddlePosition = 0; // TODO SET
                public static final double elevatorTopPosition = 0; // TODO SET
                public static final double elevatorHumanPlayerPosition = 0; // TODO SET

        }

        public static class ConeIntakeConstants {
                public static final int leftPivotMotorID = 18;
                public static final int rightPivotMotorID = 19;
                public static final int intakeMotorID = 20;

                public static final double pivotKp = 0.3;
                public static final double pivotKi = 0.0;
                public static final double pivotKd = 0.0;

                public static final double intakeSpeed = 0.3;
        }

        public static class IntakeConstants {
                public static final int pivotIntakeID = 13;
                public static final int intakeMotorID = 14;

                public static final double intakeKp = 0.045; // TODO SET
                public static final double intakeKi = 0.0;
                public static final double intakeKd = 0.0;

                public static final double minIntakePosition = 8; //TODO SET
                public static final double maxIntakePosition = 90; //TODO SET

                public static final double intakeHomedPosition = 32; //TODO SET
                public static final double intakeExtendedPosition = 65; //TODO SET

                public static final double pivotPositionConversionFactor = 100;
                public static final double pivotZeroOffset = 90.0467753; // TODO SET

                public static final double intakeSpeed = 0.7;

                public static final int beamBrakePort = 0;
        }

        public static class IndexerConstants {
                public static final int indexerMotorID = 17;

                public static final double indexerSpeed = 0.2;
        }
}
