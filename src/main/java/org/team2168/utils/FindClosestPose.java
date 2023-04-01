// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.utils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.team2168.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class FindClosestPose {

    public static List<Pose3d> list = Constants.AprilTagPoses.apriltagPoses;

    public static void main(String args[]) {
        System.out.println(findClosest(list, new Pose2d(new Translation2d(3.26, 6.91), Rotation2d.fromDegrees(-17.28))));

    }

    /**
     * 
     * @param array       List of apriltag poses to track
     * @param currentPose current pose of the robot
     * @return
     */
    public static Pose3d findClosest(List<Pose3d> array, Pose2d currentPose) {

        Pose3d finaL = new Pose3d();
        double targetNumber = currentPose.getY();

        List<Double> list = new ArrayList<Double>();

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
}