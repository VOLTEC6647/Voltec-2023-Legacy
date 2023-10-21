/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647.util.shuffleboard;

import com.andromedalib.robot.BaseTelemetryManager;
import com.team6647.subsystems.ElevatorSubsystem.ElevatorPositionState;
import com.team6647.util.AutoUtils;
import com.team6647.util.Constants.DriveConstants;
import com.team6647.util.shuffleboard.AutoModeSelector.AutoSelection;
import com.team6647.util.shuffleboard.GridPlacementSelector.GridPlacement;

/**
 * Handles selectors
 */
public class TelemetryManager extends BaseTelemetryManager {

    private static TelemetryManager instance;

    private ShuffleboardManager interactions;
    private AutoModeSelector autoSelector;
    private GridPlacementSelector gridSelector;

    /**
     * Private Constructor
     */
    private TelemetryManager() {

    }

    public static TelemetryManager getInstance() {
        if (instance == null) {
            instance = new TelemetryManager();
        }
        return instance;
    }

    @Override
    public void initTelemetry() {
        interactions = ShuffleboardManager.getInstance();
        gridSelector = new GridPlacementSelector();
        autoSelector = new AutoModeSelector();

    }

    public void setEventMap() {
        DriveConstants.eventMap.put("intakeIn", AutoUtils.autoIntakeConeSequence());
        DriveConstants.eventMap.put("intakeHome", AutoUtils.teleopHomeSequence());

        switch (getGridPlacement()) {
            case Bottom:
                DriveConstants.eventMap.put("placeCone", AutoUtils.autoPlaceConeSequence(ElevatorPositionState.BOTTOM));
                DriveConstants.eventMap.put("placeCube", AutoUtils.autoPlaceCubeSequence(ElevatorPositionState.BOTTOM));
                break;

            case Middle:
                DriveConstants.eventMap.put("placeCone", AutoUtils.autoPlaceConeSequence(ElevatorPositionState.MID));
                DriveConstants.eventMap.put("placeCube", AutoUtils.autoPlaceCubeSequence(ElevatorPositionState.MID));
                break;
            case Top:
                DriveConstants.eventMap.put("placeCone", AutoUtils.autoPlaceConeSequence(ElevatorPositionState.MAX));
                DriveConstants.eventMap.put("placeCube", AutoUtils.autoPlaceCubeSequence(ElevatorPositionState.MAX));
                break;
        }
    }

    @Override
    public void updateTelemetry() {
        interactions.updateTelemetry();
    }

    public GridPlacement getGridPlacement() {
        return gridSelector.getSelection();
    }

    public AutoSelection getAutoSelection() {
        return autoSelector.getAutoMode();
    }

}
