// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.drivetrain;

import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetOdometry extends CommandBase {
  /** Creates a new ResetOdometry. */
  Drivetrain drivetrain;
  Pose2d initRobotPose;
  public ResetOdometry(Drivetrain drivetrain, Pose2d initRobotPose) {
    this.drivetrain = drivetrain;
    this.initRobotPose = initRobotPose;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.resetOdometry(initRobotPose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return initRobotPose == drivetrain.getPose();
  }
}
