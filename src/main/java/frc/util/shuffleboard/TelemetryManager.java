/**
 * Written by Juan Pablo Gutiérrez
 */

package frc.util.shuffleboard;

import com.andromedalib.robot.BaseTelemetryManager;

public class TelemetryManager extends BaseTelemetryManager {
    private static TelemetryManager instance;

    private TelemetryManager() {

    }

    public static TelemetryManager getInstance() {
        if (instance == null) {
            instance = new TelemetryManager();
        }
        return instance;
    }
}
