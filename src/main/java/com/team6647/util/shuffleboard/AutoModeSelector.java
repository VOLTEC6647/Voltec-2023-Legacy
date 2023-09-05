/**
 * Written by Juan Pablo Gutiérrez
 * 05 09 2023
 */

package com.team6647.util.shuffleboard;

import com.team6647.util.Constants.ShuffleboardConstants;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutoModeSelector {
    private SendableChooser<AutoSelection> autoChooser = new SendableChooser<>();

    private enum AutoSelection {
        DafultMode,
    }

    public AutoModeSelector() {
        autoChooser.setDefaultOption("Default mode", AutoSelection.DafultMode);

        ShuffleboardConstants.kShuffleboardTab.add("Auto Mode", autoChooser);
    }

    public AutoSelection getAutoMode(){
        return autoChooser.getSelected();
    }
}
