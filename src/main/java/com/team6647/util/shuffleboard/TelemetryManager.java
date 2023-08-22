/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647.util.shuffleboard;

import com.andromedalib.robot.BaseTelemetryManager;

/**
 * Handles selectors
 */
public class TelemetryManager extends BaseTelemetryManager {

    private static TelemetryManager instance;

    private ShuffleboardManager interactions;

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
    }

    @Override
    public void updateTelemetry() {
        interactions.updateTelemetry();
    }
}
