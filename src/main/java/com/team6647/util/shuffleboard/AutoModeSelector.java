/**
 * Written by Juan Pablo Gutiérrez
 * 05 09 2023
 */

package com.team6647.util.shuffleboard;

import com.team6647.util.Constants.ShuffleboardConstants;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutoModeSelector {
    private SendableChooser<AutoSelection> autoChooser = new SendableChooser<>();

    public enum AutoSelection {
        Top,
        Middle,
        Bottom,
        OnlyPiece,
        DoNothimg,
        LeaveCommunity
    }

    public AutoModeSelector() {
        autoChooser.setDefaultOption("Top auto", AutoSelection.Top);
        autoChooser.setDefaultOption("Middle auto", AutoSelection.Top);
        autoChooser.setDefaultOption("Bottom auto", AutoSelection.Top);
        autoChooser.setDefaultOption("Only Piece auto", AutoSelection.OnlyPiece);
        autoChooser.setDefaultOption("Do nothing auto", AutoSelection.DoNothimg);
        autoChooser.setDefaultOption("Leave Community auto", AutoSelection.LeaveCommunity);

        ShuffleboardConstants.kShuffleboardTab.add("Auto Mode", autoChooser);
    }

    public AutoSelection getAutoMode(){
        return autoChooser.getSelected();
    }
}
