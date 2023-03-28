// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.utils;

import org.team2168.Constants;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Limelight;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class PoseEstimator {

    private Limelight lime;
    private Drivetrain drive;
    public Matrix<N3, N1> stateStandardDeviations;
    public Matrix<N3, N1> visionStandardDeviations;
    private DifferentialDrivePoseEstimator poseEstimator;

    public PoseEstimator(Limelight lime, Drivetrain drive, Matrix<N3, N1> stateStandardDeviations,
            Matrix<N3, N1> visionStandardDeviations) {
        this.lime = lime;
        this.drive = drive;
        this.stateStandardDeviations = stateStandardDeviations;
        this.visionStandardDeviations = visionStandardDeviations;
        poseEstimator = new DifferentialDrivePoseEstimator(Constants.Drivetrain.kDriveKinematics, drive.getRotation2d(),
                drive.getLeftEncoderDistance(), drive.getRightEncoderDistance(), drive.getPose(),
                stateStandardDeviations, visionStandardDeviations);
    }

    public PoseEstimator(Limelight lime, Drivetrain drive) {
        this.lime = lime;
        this.drive = drive;
        poseEstimator = new DifferentialDrivePoseEstimator(Constants.Drivetrain.kDriveKinematics, drive.getRotation2d(),
                drive.getLeftEncoderDistance(), drive.getRightEncoderDistance(), drive.getPose());
    }

    public void update() {
        if (lime.hasTarget()) {
            poseEstimator.addVisionMeasurement(
                    lime.getPose2d(),
                    Timer.getFPGATimestamp() - Units.millisecondsToSeconds(lime.getLatencyMs())
                            - Units.millisecondsToSeconds(lime.getCapturedLatencyTime()));

            poseEstimator.update(drive.getRotation2d(), drive.getLeftEncoderDistance(),
                    drive.getRightEncoderDistance());
        } else if (!lime.hasTarget()) {
            poseEstimator.resetPosition(drive.getRotation2d(), drive.getLeftEncoderDistance(),
                    drive.getRightEncoderDistance(), drive.getPose());
        }
    }

    public void setVisionDeviations(Matrix<N3, N1> newVisionDeviations) {
        poseEstimator.setVisionMeasurementStdDevs(newVisionDeviations);
    }

    public void addVisionMeasurement(Pose2d visionPoseMeters) {
        poseEstimator.addVisionMeasurement(visionPoseMeters,
                Timer.getFPGATimestamp() - Units.millisecondsToSeconds(lime.getLatencyMs())
                        - Units.millisecondsToSeconds(lime.getCapturedLatencyTime()));
    }

    public Pose2d getEstimatedPosition() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPosition(Pose2d resetPose) {
        poseEstimator.resetPosition(drive.getRotation2d(), drive.getLeftEncoderDistance(),
                drive.getRightEncoderDistance(), resetPose);
    }

}
