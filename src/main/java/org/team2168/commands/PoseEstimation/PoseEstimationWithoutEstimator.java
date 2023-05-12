// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.PoseEstimation;

import org.team2168.Constants;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Limelight;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PoseEstimationWithoutEstimator extends CommandBase {

  public Limelight lime;
  public LinearFilter filter; // this filter performs exponential smoothing to calulate the moving (weighted)
                              // average
                              // between gyro and vision pose estimation
  public Drivetrain drive;
  public Pose3d weightedPose;

  Matrix<N3, N1> stateStdDeviations;
  Matrix<N3, N1> visionStdDeviations;

  public PoseEstimationWithoutEstimator(Limelight lime, Drivetrain drive) {
    this.lime = lime;
    this.drive = drive;

    filter = LinearFilter.singlePoleIIR(0.1,
        Units.millisecondsToSeconds(lime.getLatencyMs() + lime.getCapturedLatencyTime()));
  }

  public PoseEstimationWithoutEstimator(Limelight lime, Drivetrain drive, Matrix<N3, N1> stateStdDeviations,
      Matrix<N3, N1> visionStdDeviations) {
    this.lime = lime;
    this.drive = drive;

    filter = LinearFilter.singlePoleIIR(0.1,
        Units.millisecondsToSeconds(lime.getLatencyMs() + lime.getCapturedLatencyTime()));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lime.enableBaseCameraSettings();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // this gets an weighted average of the camera estimated apriltag pose, as well
    // as the actual position of the apriltag in space.
    double aprilTagPoseAverageX = filter.calculate(
        lime.getApriltagDimensionsFromFidicualId().getX() + lime.getAprilTagPoseRelativeToLimelight().getX());
    double aprilTagPoseAverageY = filter.calculate(
        lime.getApriltagDimensionsFromFidicualId().getY() + lime.getAprilTagPoseRelativeToLimelight().getY());
    double aprilTagPoseAverageZ = filter.calculate(
        lime.getApriltagDimensionsFromFidicualId().getZ() + lime.getAprilTagPoseRelativeToLimelight().getZ());
    double apriltagPoseAverageRoll = filter.calculate(lime.getApriltagDimensionsFromFidicualId().getRotation().getX()
        + lime.getAprilTagPoseRelativeToLimelight().getRotation().getX());
    double apriltagPoseAveragePitch = filter.calculate(lime.getApriltagDimensionsFromFidicualId().getRotation().getY() +
        lime.getAprilTagPoseRelativeToLimelight().getRotation().getY());
    double apriltagPoseAverageYaw = filter.calculate(lime.getApriltagDimensionsFromFidicualId().getRotation().getZ() +
        lime.getAprilTagPoseRelativeToLimelight().getRotation().getZ());

    // this calculates the weighted average of each pose3d component, in the x y z
    // plane. The horizontal distance point is calculated by getting the difference
    // between x2, and x1. This essentially forms a right triangle between the
    // apriltag, and estimated camerapose, we then fuse this vision data with data
    // reported by the gyro to get a weighted average of the (x, y, z, roll, pitch,
    // yaw)
    double weightedAverageX = filter.calculate(
        (aprilTagPoseAverageX - lime.getPose3d().getX()) + drive.getPose().getX());
    double weightedAverageY = filter
        .calculate((aprilTagPoseAverageY - lime.getPose3d().getY()) + drive.getPose().getY());
    double weightedAverageZ = filter
        .calculate((aprilTagPoseAverageZ - lime.getPose3d().getZ()) + drive.getPose().getY());
    double weightedAverageRoll = filter
        .calculate(apriltagPoseAverageRoll - lime.getPose3d().getRotation().getX() + drive.getRoll());
    double weightedAveragePitch = filter
        .calculate(apriltagPoseAveragePitch - lime.getPose3d().getZ() + drive.getPitch());
    double weightedAverageYaw = filter.calculate(
        apriltagPoseAverageYaw - lime.getPose3d().getRotation().getZ() + drive.getPose().getRotation().getDegrees());

    weightedPose = new Pose3d(new Translation3d(weightedAverageX, weightedAverageY, weightedAverageZ),
        new Rotation3d(weightedAverageRoll, weightedAveragePitch, weightedAverageYaw));

    if (lime.hasTarget()) {
      drive.resetOdometry(weightedPose.toPose2d());
    } else if (!lime.hasTarget() && RobotState.isTeleop()) {
      drive.zeroHeading();

    }

    

  }

  public Pose3d getWeightedPose() {
    return weightedPose;
  }

  public Pose2d getPoseDisplayedField2d() {
    Constants.FieldMetrics.field.setRobotPose(getWeightedPose().toPose2d());
    return Constants.FieldMetrics.field.getRobotPose();
}

 
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}