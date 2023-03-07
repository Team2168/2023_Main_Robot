// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.AutoAlignWithPoseEstimation;

import org.team2168.Constants;
import org.team2168.commands.PoseEstimation.PoseEstimationWeightedAverage;
import org.team2168.commands.PoseEstimation.PoseEstimationWithLimelight;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Limelight;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoAlignWithPoseEstimation extends CommandBase {

  public enum ScoringArea {
    LOW_NODE(new Pose3d()),
    MIDDLE_NODE(new Pose3d()),
    HIGH_NODE(new Pose3d());

    public final Pose3d nodePoses;

    private ScoringArea(Pose3d nodePoses) {
      this.nodePoses = nodePoses;
    }

  }

  public ProfiledPIDController xController;
  public ProfiledPIDController yController;
  public ProfiledPIDController turnController;
  public Constraints xControllerConstraints;
  public Constraints yControllerConstraints;
  public Constraints turnControllerConstraints;
  public PoseEstimationWeightedAverage poseEstimation;
  public Pose3d robotPose;
  public Limelight lime;
  public Drivetrain drive;

  public final Pose3d POSE_TO_TAG_LIMIT;

  public AutoAlignWithPoseEstimation(Drivetrain drive, Limelight lime, ScoringArea scoringArea) {
    xControllerConstraints = new Constraints(Constants.Drivetrain.kMaxSpeedMetersPerSecond,
        Constants.Drivetrain.kMaxAccelerationMetersPerSecondSquared); // change to realistic constraint values;
    yControllerConstraints = new Constraints(Constants.Drivetrain.kPDriveVel,
        Constants.Drivetrain.kMaxAccelerationMetersPerSecondSquared);
    turnControllerConstraints = new Constraints(1.0, 1.0); // change to real values

    xController = new ProfiledPIDController(1.0, 0, 0, xControllerConstraints);
    yController = new ProfiledPIDController(1.0, 0, 0, yControllerConstraints);
    turnController = new ProfiledPIDController(1.0, 0, 0, null);
    xController.setTolerance(0.1);
    yController.setTolerance(0.0);
    this.drive = drive;
    this.lime = lime;
    poseEstimation = new PoseEstimationWeightedAverage(lime, drive);
    robotPose = poseEstimation.getWeightedPose();

    POSE_TO_TAG_LIMIT = scoringArea.nodePoses;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    var goalPose = lime.getTagPose().transformBy(new Transform3d(lime.getTagPose(), POSE_TO_TAG_LIMIT)).toPose2d();

    xController.setGoal(goalPose.getX());
    yController.setGoal(goalPose.getY());
    turnController.setGoal(goalPose.getRotation().getDegrees());

    var xSpeed = xController.calculate(robotPose.getX());
    var ySpeed = yController.calculate(robotPose.getY());
    var rotSpeed = turnController.calculate(robotPose.getRotation().getZ());

    ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, drive.getRotation2d());

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
