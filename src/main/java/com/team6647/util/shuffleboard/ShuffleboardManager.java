package com.team6647.util.shuffleboard;

import com.andromedalib.andromedaSwerve.utils.shufflleboard.tabs.AndromedaSwerveInfo;
import com.andromedalib.shuffleboard.ShuffleboardTabBase;
import com.team6647.util.Constants.ShuffleboardConstants;
import com.team6647.util.shuffleboard.tabs.DebugTab;
import com.team6647.util.shuffleboard.tabs.DefaultTab;

/**
 * Base Shufflebard manager class. It is called by default by the
 * {@link TelemetryManager} class. Update methods to include your subsystems
 * Handles only data related to shuffleboard
 */
public class ShuffleboardManager {
    private static ShuffleboardManager instance;

    private static ShuffleboardTabBase[] tabs;

    public boolean debug = true;

    /**
     * Private Constructor
     */
    private ShuffleboardManager() {
        if (debug) {
            tabs = new ShuffleboardTabBase[] {
                new DebugTab(ShuffleboardConstants.kShuffleboardTab),
                //new AndromedaSwerveInfo(ShuffleboardConstants.kShuffleboardTab, false),
            };
        } else {
            tabs = new ShuffleboardTabBase[] {
                new AndromedaSwerveInfo(ShuffleboardConstants.kShuffleboardTab, false),
                new DefaultTab(ShuffleboardConstants.kShuffleboardTab),
            };
        }
    }

    /**
     * Initializes a new {@link ShuffleboardManager}
     * 
     * @return {@link ShuffleboardManager} singleton instance
     */
    public static ShuffleboardManager getInstance() {
        if (instance == null) {
            instance = new ShuffleboardManager();
        }
        return instance;
    }

    public void updateTelemetry() {
        if (tabs != null) {
            for (ShuffleboardTabBase tab : tabs) {
                tab.updateTelemetry();
            }
        }
    }
}
