// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.PoseEstimation;

import org.team2168.Constants;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Limelight;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PoseEstimationWithLimelight extends CommandBase {
  private Limelight lime;
  private Drivetrain drivetrain;
  private DifferentialDrivePoseEstimator poseEstimator;
  private Matrix<N3, N1> standardDeviations;
  private Matrix<N3, N1> visionDeviations;

  private Matrix<N3, N1> stateStandardDeviations = VecBuilder.fill(0.02, 0.02, 0.01); // standard base values used by
                                                                                      // wpilibs.
  private Matrix<N3, N1> visionStandardDeviations = VecBuilder.fill(0.1, 0.1, 0.1); // standard base values used by
                                                                                    // wpilibs.

  public PoseEstimationWithLimelight(Limelight lime, Drivetrain drivetrain) {

    this.drivetrain = drivetrain;
    this.lime = lime;

    poseEstimator = new DifferentialDrivePoseEstimator(Constants.Drivetrain.kDriveKinematics,
        drivetrain.getRotation2d(), drivetrain.getLeftEncoderDistance(),
        drivetrain.getRightEncoderDistance(), drivetrain.getPose(), stateStandardDeviations, visionStandardDeviations);
  }

  public PoseEstimationWithLimelight(Limelight lime, Drivetrain drivetrain,
      Matrix<N3, N1> standardDeviations, Matrix<N3, N1> visionDeviations) {

    this.lime = lime;
    this.drivetrain = drivetrain;
    this.standardDeviations = standardDeviations;
    this.visionDeviations = visionDeviations;

    poseEstimator = new DifferentialDrivePoseEstimator(Constants.Drivetrain.kDriveKinematics,
        drivetrain.getRotation2d(), drivetrain.getLeftEncoderDistance(), drivetrain.getRightEncoderDistance(),
        drivetrain.getPose(), standardDeviations, visionDeviations);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    poseEstimator.addVisionMeasurement(
        new Pose2d(new Translation2d(lime.botPoseArrayTwo[1], lime.botPoseArrayTwo[3]),
            Rotation2d.fromDegrees((lime.botPoseArrayTwo[6]))),
        Timer.getFPGATimestamp() - (lime.getLatencyMs() / 1000) - (lime.getCapturedLatencyTime() / 1000));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    poseEstimator.update(drivetrain.getRotation2d(), drivetrain.getLeftEncoderDistance(),
        drivetrain.getRightEncoderDistance());
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