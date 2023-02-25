// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.PoseEstimation;

import org.team2168.subsystems.Limelight;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PoseEstimationWithLimelight extends CommandBase {
  private Limelight lime;
  private DifferentialDrivePoseEstimator poseEstimator  = new DifferentialDrivePoseEstimator(null, null, 0, 0, null);
  private DifferentialDrivePoseEstimator estimator = new DifferentialDrivePoseEstimator(null, null, 0, 0, null, null, null)
  public PoseEstimationWithLimelight(Limelight lime) {
    this.lime = lime;
    // Use addRequirements() here to declare subsystem dependencies.
    poseEstimator.addVisionMeasurement(new Pose2d(new Translation2d(lime.botPoseArrayTwo[1], lime.botPoseArrayTwo[2]), new Rotation2d(Units.radiansToDegrees(lime.getOffsetX()))), Timer.getFPGATimestamp());
    poseEstimator.setVisionMeasurementStdDevs(null);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
