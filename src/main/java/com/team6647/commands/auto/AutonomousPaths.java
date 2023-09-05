/**
 * Written by Juan Pablo Gutiérrez
 * 03 09 2023
 */

package com.team6647.commands.auto;

import com.andromedalib.andromedaSwerve.commands.SwerveDriveCommand;
import com.andromedalib.andromedaSwerve.systems.AndromedaSwerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutonomousPaths {

    public Command defaultCommand() {
        return Commands.sequence(
                new SwerveDriveCommand(AndromedaSwerve.getInstance(null), () -> 0.0, () -> 0.5, () -> 0.0,
                        () -> true))
                .withTimeout(2);

    }
    /*
     * public Command topCubeCone() {
     * return null;
     * }
     * 
     * public Command middleBalance(){
     * Commands.sequence(new RunCommand(null, null))
     * }
     */
}
