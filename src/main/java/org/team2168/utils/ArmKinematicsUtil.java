// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.utils;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class ArmKinematicsUtil {

    private Translation2d endPosition;
    private double x;
    private double y;
    private double angle1;
    private double angle2;
    private double armJointOffset = 3.6;
    private double elbowJointOffset = 57;
    private double armLengthMeters = Units.inchesToMeters(24);
    private double elbowArmLengthMeters = Units.inchesToMeters(24);
    public static double elbowAngle;

    public static double bicepAngle;

    // * */

    public ArmKinematicsUtil(double x, double y, double angle1, double angle2) {
        endPosition = new Translation2d(x, y);
        this.x = x;
        this.y = y;
        this.angle1 = angle1;
        this.angle2 = angle2;
    }

    public double[] inverse() {
        double[] list = { calculateBicepAngle(), calculateElbowAngle() };
        return list;
    }

    public double calculateBicepAngle() {
        bicepAngle = Units.radiansToDegrees(Math.atan(endPosition.getY() / endPosition.getX())
                - Math.atan((elbowArmLengthMeters * Math.sin(calculateElbowAngle()))
                        / (armLengthMeters + elbowArmLengthMeters * Math.cos(calculateElbowAngle()))));

        if (!Double.isNaN(bicepAngle)) {
            return bicepAngle;
        } else {
            throw new Error("unusable angle for joint");
        }
    }

    public double calculateElbowAngle() {

        elbowAngle = Units.radiansToDegrees(Math
                .acos((Math.pow(endPosition.getX(), 2) + Math.pow(endPosition.getY(), 2) - Math.pow(armLengthMeters, 2)
                        - Math.pow(elbowArmLengthMeters, 2)) / (2 * armLengthMeters * elbowArmLengthMeters)));

        if (!Double.isNaN(elbowAngle)) {
            return elbowAngle;
        } else {
            throw new Error("value is unusable for joint angle");
        }
    }

    public Translation2d forward() {
        Vector<N2> vector = VecBuilder.fill(angle1, angle2);
        Vector<N2> endPositionMatrix = VecBuilder.fill(x, y);
        double x = armLengthMeters * Math.cos(vector.get(0, 0))
                + elbowArmLengthMeters * Math.cos(vector.get(0, 0) + vector.get(1, 0));
        double y = armLengthMeters * Math.sin(vector.get(0, 0))
                + elbowArmLengthMeters * Math.sin(vector.get(0, 0) + vector.get(1, 0));
        return new Translation2d(x, y);
    }

    public void simulateAngles() {
        Vector<N2> vector = VecBuilder.fill(+angle1, +angle2);

    }
}
