// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.PoseEstimation;

import org.team2168.Constants;
import org.team2168.commands.PoseEstimation.PoseEstimationWithLimelight;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Turret;
import org.team2168.utils.FindClosestPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetTurretOdometry extends CommandBase {

  public Limelight lime;
  public Turret turret;
  public Drivetrain drive;
  public boolean turnTurretToHighNode;
  double diffX;
  double diffY;
  double finalAngle;
  public Pose3d apriltagPose;

  public ResetTurretOdometry(Limelight lime, Turret turret,
      boolean turnTurretToHighNode, Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.lime = lime;
    this.turret = turret;
    this.turnTurretToHighNode = turnTurretToHighNode;
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lime.enableBaseCameraSettings();
    lime.enableVision(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (lime.hasTarget()) {
      apriltagPose = lime.getAprilTagPoseRelativeToLimelight();
    } else {
      apriltagPose = FindClosestPose.findClosest(Constants.AprilTagPoses.apriltagPoses, drive.getPose());
    }
    // make code that checks what apriltag I.D we are at based on pose with switch
    // case, or if else since the limelight won't look at an I.D.
    if (turnTurretToHighNode == false) {
      diffX = Math.abs(drive.getPose().getX() - apriltagPose.getX());
      diffY = Math.abs(drive.getPose().getY() - apriltagPose.getY());

      // Pose2d relativePose =
      // poseEstimator.relativeTo(lime.getApriltagDimensionsFromFidicualId().toPose2d());

      // Pose2d poseRelativeToTag = lime.getPoseInTargetSpace().toPose2d();

      finalAngle = turret.getTurretAngle() + Units.radiansToDegrees(Math.atan(diffY / diffX));

      turret.setRotationDegrees(finalAngle);

    } else if (turnTurretToHighNode == true) {
      Pose3d middleToHighNodeTransform = apriltagPose.plus(
          new Transform3d(new Translation3d(0.66, 0.3048, 0.0), new Rotation3d(0.0, 0.0, 0.0)));

      diffX = Math.abs(drive.getPose().getX() - middleToHighNodeTransform.getX());
      diffY = Math.abs(drive.getPose().getY() - middleToHighNodeTransform.getY());
      finalAngle = turret.getTurretAngle() + Units.radiansToDegrees(Math.atan(diffY / diffX));

      turret.setRotationDegrees(finalAngle);

    }
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

