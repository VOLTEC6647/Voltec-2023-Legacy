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

                public static final double elevatorFloorPosition = 0.0;
                public static final double elevatorHomedPosition = 1;
                public static final double elevatorBottomPosition = 13.3;
                public static final double elevatorMiddlePosition = 22.19;
                public static final double elevatorTopPosition = 38.71;
                public static final double elevatorHumanPlayerPosition = 20;

                public static final float minElevatorSoftLimit = 0;
                public static final float maxElevatorSoftLimit = (float) elevatorTopPosition;

                public static final double elevatorEncoderPositionConversionFactor = 100;
                public static final double elevatorEncoderZeroOffset = 0.0;
                public static final boolean elevatorEncoderInverted = true;

        }

        public static class ArmIntakeConstants {
                public static final int armMotor1ID = 17;
                public static final int armMotor2ID = 18;
                public static final int intakeMotorID = 20;

                public static final double pivotKp = 0.0035;
                public static final double pivotKi = 0.0000000012;
                public static final double pivotKd = 0.0;

                public static final double intakeSpeed = 0.3;
                public static final int beamBrakePort = 2;

                public static final double intakeHomedPosition = 200;
                public static final double intakeFloorPosition = 88;
                public static final double intakePlacingPositon = 215;
                public static final double intakeScoringPositon = 200;
                public static final double intakeIndexingPosition = 248;
                public static final double intakeHumanPlayerPosition = 205;

                public static final double armEncoderPositionConversionFactor = 360;
                public static final double armEncoderZeroOffset = 150.982;
                public static final boolean armEncoderInverted = false;
        }

        public static class IntakeConstants {
                public static final int pivotIntakeID = 13;
                public static final int intakeMotorID = 14;

                public static final double intakeKp = 0.02;
                public static final double intakeKi = 0.0;
                public static final double intakeKd = 0.0;

                public static final double intakeHomedPosition = 39;
                public static final double intakeExtendedPosition = 59;

                public static final double minIntakePosition = intakeHomedPosition + 1;
                public static final double maxIntakePosition = intakeExtendedPosition + 1;

                public static final double pivotPositionConversionFactor = 100;
                public static final double pivotZeroOffset = 90.0467753;

                public static final double intakeSpeed = 0.7;

                public static final int beamBrakePort = 0;
        }

        public static class VisionConstants {
                public static final double kPdrive = 0.01;
                public static final double kIdrive = 0.0;
                public static final double kDdrive = 0.0;

                public static final double kPstrafe = 0.01;
                public static final double kIstrafe = 0.0;
                public static final double kDstrafe = 0.0;

                public static final double kProt = 0.01;
                public static final double kIrot = 0.0;
                public static final double kDrot = 0.0;

                public static final int aprilLimePipe = 0;
                public static final int retroLimePipe = 2;
        }
}
