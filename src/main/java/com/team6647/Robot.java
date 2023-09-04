package com.team6647;

import com.andromedalib.robot.SuperRobot;
import com.pathplanner.lib.server.PathPlannerServer;
import com.team6647.util.shuffleboard.TelemetryManager;

public class Robot extends SuperRobot {

    private RobotContainer container;

    @Override
    public void robotInit() {
        container = RobotContainer.getInstance();
        super.setRobotContainer(container, TelemetryManager.getInstance(), false);

        super.robotInit();

        PathPlannerServer.startServer(5811); 
    } 
}
