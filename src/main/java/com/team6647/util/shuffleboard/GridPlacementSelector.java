/**
 * Written by Juan Pablo Gutierrez
 * 13 10 2023
 */
package com.team6647.util.shuffleboard;

import com.team6647.util.Constants.ShuffleboardConstants;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class GridPlacementSelector {

    public enum GridPlacement {
        Bottom,
        Middle,
        Top
    }

    private SendableChooser<GridPlacement> gridSelector = new SendableChooser<>();

    public GridPlacementSelector() {
        gridSelector.setDefaultOption("Bottom", GridPlacement.Bottom);
        gridSelector.addOption("Middle", GridPlacement.Middle);
        gridSelector.addOption("Top", GridPlacement.Top);

        ShuffleboardConstants.kShuffleboardTab.add("Grid Placement", gridSelector).withPosition(7, 0);
    }

    public GridPlacement getSelection() {

        return gridSelector.getSelected();
    }
}
