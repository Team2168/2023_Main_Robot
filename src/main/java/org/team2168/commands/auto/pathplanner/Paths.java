// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto.pathplanner;

import java.io.IOException;

import org.team2168.Constants;
import org.team2168.utils.PathUtil;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class Paths {
    public final Trajectory path_example;
    public final Trajectory path_left_leave_community_blue;
    public final Trajectory path_left_leave_community_red;
    public final Trajectory path_4_m_forward;
    public final Trajectory path_score_low_1;
    public final Trajectory path_score_low_2;
    public final Trajectory toMidNode;
    public final Trajectory toChargingStation;

    private static Paths instance = null;

    private Paths() {
        System.out.println("******* Begin generating autos *******");

        path_example = getTrajectory("example_path", true);
        path_left_leave_community_blue = getTrajectory("Left community", true);
        path_left_leave_community_red = getTrajectory("Right community", true);
        path_4_m_forward = getTrajectory("FWD_1_M", false);
        path_score_low_1 = getTrajectory("score_low_wne_1", false);
        path_score_low_2 = getTrajectory("score_low_wne_2", false);
        toMidNode = getTrajectory("ToMidNode", false);
        toChargingStation = getTrajectory("ToChargingStation", false);

        System.out.println("******* Finish generating autos *******");
    }

    private Trajectory getTrajectory(String pathname, boolean reversed, double vel, double accel) {
        try {
            return PathUtil.getPathPlannerTrajectory(pathname, reversed, vel, accel);
        } catch (IOException e) {
            final String ERRORMESSAGE = "Failed to read path %s! Check the file name. Falling back to empty trajectory.";
            DriverStation.reportError(String.format(ERRORMESSAGE, pathname), e.getStackTrace());
            return new Trajectory();
        }
    }

    private Trajectory getTrajectory(String pathname, boolean reversed) {
        return this.getTrajectory(pathname, reversed, Constants.Drivetrain.kvVoltSecondsPerMeter, Constants.Drivetrain.kaVoltSecondsSquaredPerMeter);
    }

    public static Paths getInstance() {
        if (instance == null) {
            instance = new Paths();
        }
        return instance;
    }

}
