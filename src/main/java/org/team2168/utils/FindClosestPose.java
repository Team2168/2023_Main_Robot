// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.utils;

import org.team2168.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.*;

/** Add your docs here. */
public class FindClosestPose {

    public static List<Pose3d> list = Constants.AprilTagPoses.apriltagPoses;
    private static Pose2d pose = new Pose2d(6.48305939385093, 4.3837593983745803, Rotation2d.fromDegrees(0.2384756393));

    public static void main(String args[]) {
        System.out.println(findClosest(Constants.AprilTagPoses.apriltagPoses, pose));
        System.out.println(getLineOfHeading(pose, pose.getRotation().getDegrees()));
        

    }

    /**
     * 
     * @param array       List of apriltag poses to track
     * @param currentPose current pose of the robot
     * @return the appropiate apriltag Pose3d the algorithm thinks we tracked based
     *         on robot odometry
     */
    public static Pose3d findClosest(List<Pose3d> array, Pose2d currentPose) {

        Pose3d finaL = new Pose3d();
        double targetNumber = currentPose.getY();

        List<Double> list = new ArrayList<>();

        try {
            for (int i = 0; i < array.size(); i++) {
                double x = Math.abs(targetNumber - array.get(i).getY());
                list.add(x);
            }

            Collections.sort(list);
            double formatX = limitDecimalPlaces(Math.abs(-list.get(0) + targetNumber), 3);
            double formatY = limitDecimalPlaces(list.get(0) + targetNumber, 3);

            if (currentPose.getX() > 8.25) {
                for (int j = 0; j < list.size(); j++) {
                    if ((formatX == array.get(j).getY())) {
                        return finaL = array.get(j);
                    } else if ((formatY == array.get(j).getY())) {
                        return finaL = array.get(j);

                    }
                }
            } else {
                for (int n = list.size() - 1; n > 0; n--) {
                    if ((formatX == array.get(n).getY())) {
                        return finaL = array.get(n);
                    } else if ((formatY == array.get(n).getY())) {
                        return finaL = array.get(n);

                    }
                }
            }

        } catch (IndexOutOfBoundsException | UnsupportedOperationException e) {
            e.printStackTrace();
        }

        return finaL;

    }

    public static double limitDecimalPlaces(double num, int n) {
        double factor = Math.pow(10, n);
        return Math.round(num * factor) / factor;
    }

    public static double getLineOfHeading(Pose2d currentPose, double x) {
        double m = Math.tan(currentPose.getRotation().getDegrees());

        System.out.println(m);
        System.out.println("y = " + m + "x + " + currentPose.getY());
        return (m * x) + currentPose.getY();

    }

}