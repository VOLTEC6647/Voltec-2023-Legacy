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
        OnlyPieceCone,
        OnlyPieceCube,
        DoNothimg,
        LeaveCommunity
    }

    public AutoModeSelector() {
        autoChooser.setDefaultOption("Top auto", AutoSelection.Top);
        autoChooser.setDefaultOption("Middle auto", AutoSelection.Middle);
        autoChooser.setDefaultOption("Bottom auto", AutoSelection.Bottom);
        autoChooser.setDefaultOption("Only Piece Cone auto", AutoSelection.OnlyPieceCone);
        autoChooser.setDefaultOption("Only Piece Cube auto", AutoSelection.OnlyPieceCube);
        autoChooser.setDefaultOption("Do nothing auto", AutoSelection.DoNothimg);
        autoChooser.setDefaultOption("Leave Community auto", AutoSelection.LeaveCommunity);

        ShuffleboardConstants.kShuffleboardTab.add("Auto Mode", autoChooser);
    }

    public AutoSelection getAutoMode(){
        return autoChooser.getSelected();
    }
}
