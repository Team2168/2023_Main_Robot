// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.utils;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import org.team2168.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Add your docs here. */
public class FindClosestPose {

    public static List<Pose3d> list = Constants.AprilTagPoses.apriltagPoses;

    public static void main(String args[]) {
        System.out.println(findClosest(list, new Pose2d(new Translation2d(3.47, 7.42), new Rotation2d())));

    }

    /**
     * 
     * @param array
     * @param targetNumber
     * @return
     */
    public static Pose3d findClosest(List<Pose3d> array, Pose2d currentPose) {

        Pose3d finaL = new Pose3d();
        double targetNumber = Math.round(currentPose.getY());
        double secondTargetNumber = Math.round(currentPose.getX());

        List<Double> list = new ArrayList<Double>();
        List<Double> xList = new ArrayList<Double>();
        try {
            for (int i = 0; i < array.size(); i++) {

                double x = Math.abs(targetNumber - array.get(i).getY());
                double y = Math.abs(secondTargetNumber - array.get(i).getX());
                xList.add(y);
                list.add(x);

            }

            Collections.sort(list);
            Collections.sort(xList);
            double formatX = limitDecimalPlaces(Math.abs(-list.get(0) + targetNumber), 3);
            double formatY = limitDecimalPlaces(list.get(0) + targetNumber, 3);
            double secondFormatX = limitDecimalPlaces(Math.abs(-xList.get(0) + secondTargetNumber), 3);
            double secondFormatY = limitDecimalPlaces(xList.get(0) + secondTargetNumber, 3);

            for (int j = 0; j < list.size(); j++) {
                for (int u = j; u < xList.size(); u++) {
                    if ((formatX == array.get(j).getY()) || (secondFormatX == array.get(u).getX())) {
                        return finaL = array.get(u);

                    } else if ((formatY == array.get(j).getY()) || (secondFormatY == array.get(u).getY())) {
                        if (secondFormatY == array.get(u).getX()) {
                            return finaL = array.get(u);
                        }
                    }
                }
            }
        } catch (IndexOutOfBoundsException | UnsupportedOperationException e) {
            System.out.println("error.... returning 1, falling back to terminal");
        }

        return finaL;

    }

    public static double limitDecimalPlaces(double num, int n) {
        double factor = Math.pow(10, n);
        return Math.round(num * factor) / factor;
    }
}