/**
 * Written by Juan Pablo Gutiérrez
 * 03 09 2023
 */

package com.team6647.commands.auto;

import com.andromedalib.andromedaSwerve.commands.SwerveDriveCommand;
import com.andromedalib.andromedaSwerve.systems.AndromedaSwerve;
import com.team6647.util.AutoUtils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutonomousPaths extends AutoUtils {

    public static Command topAuto() {
        return Commands.sequence(autoDriveSubsystem.getFullAuto("Top"));
    }

    public static Command middleAuto() {
        return Commands.sequence(
                getGridPlacement(Piece.Cube),
                new SwerveDriveCommand(
                        AndromedaSwerve.getInstance(null, null), () -> 0.0, () -> -0.7, () -> 0.0, () -> true)
                        .withTimeout(1),
                new MuBalance(AndromedaSwerve.getInstance(null, null), autoDriveSubsystem));
    }

    public static Command bottomAuto() {
        return Commands.sequence(autoDriveSubsystem.getFullAuto("Bottom"));
    }

    public static Command leaveCommunityAuto() {
        return Commands.sequence(getGridPlacement(Piece.Cube), new SwerveDriveCommand(
                AndromedaSwerve.getInstance(null, null), () -> 0.0, () -> -0.7, () -> 0.0, () -> true).withTimeout(.75),
                new SwerveDriveCommand(
                        AndromedaSwerve.getInstance(null, null), () -> 0.0, () -> -0.25, () -> 0.0, () -> true)
                        .withTimeout(2));
    }
}
