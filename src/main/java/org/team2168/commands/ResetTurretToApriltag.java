// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import org.team2168.commands.PoseEstimation.PoseEstimationWithLimelight;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Turret;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetTurretToApriltag extends CommandBase {
  
  public Limelight lime;
  public Turret turret;
  public PoseEstimationWithLimelight poseEstimator;


  
  public ResetTurretToApriltag(Limelight lime, Turret turret, PoseEstimationWithLimelight poseEstimator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.lime = lime;
    this.turret = turret;
    this.poseEstimator = poseEstimator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //make code that checks what apriltag I.D we are at based on pose with switch case, or if else.
    double diffX = poseEstimator.getPose().getX() - lime.getApriltagDimensionsFromFidicualId().getX();
    double diffY = poseEstimator.getPose().getY() - lime.getApriltagDimensionsFromFidicualId().getY();

    double finalAngle = turret.getTurretAngle() + Units.radiansToDegrees(Math.atan(diffY/diffX));

    turret.setRotationDegrees(finalAngle);
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
