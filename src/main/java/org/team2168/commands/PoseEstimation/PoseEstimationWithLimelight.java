// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.PoseEstimation;

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
  private DifferentialDrivePoseEstimator poseEstimator  = new DifferentialDrivePoseEstimator(null, null, 0, 0, null);
  private Matrix<N3,N1> matrix = VecBuilder.fill(0.01, 0.02, 0.03);

  public PoseEstimationWithLimelight(Limelight lime) {
    this.lime = lime;
    // Use addRequirements() here to declare subsystem dependencies.
    poseEstimator.addVisionMeasurement(new Pose2d(new Translation2d(lime.botPoseArrayTwo[1], lime.botPoseArrayTwo[3]), new Rotation2d(Units.radiansToDegrees(lime.botPoseArrayTwo[6]))), 
    Timer.getFPGATimestamp() - (lime.getLatencyMs() / 1000) - (lime.getCapturedLatencyTime() / 1000));

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
