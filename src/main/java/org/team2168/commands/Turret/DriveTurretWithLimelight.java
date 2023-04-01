// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Turret;

import org.team2168.Constants;
import org.team2168.Constants.AprilTagPoses;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Turret;
import org.team2168.utils.FindClosestPose;
import org.team2168.utils.Util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import io.github.oblarg.oblog.annotations.Log;

public class DriveTurretWithLimelight extends CommandBase {

  private Turret turret;
  private Limelight limelight;
  private Drivetrain drivetrain;
  
  private double errorToleranceAngle = 0.1;
  private double limeXPos;
  private double limeYPos;
  private double avg_limeXPos;
  public Pose3d apriltagPose;
  double DifferenceX;
  double DifferenceY;
  double angle;

  private double currentPos;
  private double targetPos;

  private double forwardSoftLimit;
  private double reverseSoftLimit;

  private static final double LIME_KP = 0.65;

  @Log(name = "Turn Speed")
  private double driveLimeTurn;
  /** Creates a new DriveTurretWithLimelight. */
  public DriveTurretWithLimelight(Turret turret, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.limelight = limelight;

    addRequirements(turret);
  }

  public DriveTurretWithLimelight(Turret turret, Limelight limelight, double acceptableAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.limelight = limelight;
    acceptableAngle = errorToleranceAngle;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPos = turret.getEncoderPosition();
    forwardSoftLimit = Turret.getForwardSoftLimit();
    reverseSoftLimit = Turret.getReverseSoftLimit();
    avg_limeXPos = 0.0;
    limelight.enableBaseCameraSettings();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limeXPos = limelight.getOffsetX();
    avg_limeXPos = Util.runningAverage(limeXPos, avg_limeXPos, 0.15);
    currentPos = turret.getEncoderPosition();
    targetPos = currentPos + (avg_limeXPos * LIME_KP);
    if (targetPos > reverseSoftLimit && targetPos < forwardSoftLimit) {
      driveLimeTurn = targetPos;
    }
    else if (targetPos < reverseSoftLimit) {
      driveLimeTurn = reverseSoftLimit;
    }
    else if (targetPos > forwardSoftLimit) {
      driveLimeTurn = forwardSoftLimit;
    }
  
    turret.setRotationDegrees(driveLimeTurn);

  //  if (limelight.hasTarget()) {
  //     targetPos = currentPos + (avg_limeXPos * LIME_KP);
  //   }
  //   else {
  //     //targetPos = turret.amountFromZeroToRotate(drivetrain.getInstance().getHubHeadingFromRobot());
  //   } 

    if (limelight.hasTarget()) {
      apriltagPose = limelight.getAprilTagPoseRelativeToLimelight();
    } else {
      apriltagPose = FindClosestPose.findClosest(Constants.AprilTagPoses.apriltagPoses, drivetrain.getPose());
    }

    DriverStation.Alliance driverstationColor = DriverStation.getAlliance();

    if (driverstationColor == DriverStation.Alliance.Blue){
      for(int i = 0; i <= 3; i++){
        if(AprilTagPoses.apriltagPoses.get(i).getY() >= 0){
          DifferenceX = Drivetrain.getInstance().getPose().getX() - AprilTagPoses.apriltagPoses.get(i).getX();
          DifferenceY = Drivetrain.getInstance().getPose().getY() - AprilTagPoses.apriltagPoses.get(i).getY();

          angle = Math.atan(DifferenceY/DifferenceX);
        }
        else{
          DifferenceX = Drivetrain.getInstance().getPose().getX() + AprilTagPoses.apriltagPoses.get(i).getX();
          DifferenceY = Drivetrain.getInstance().getPose().getY() + AprilTagPoses.apriltagPoses.get(i).getY();

          angle = Math.atan(DifferenceY/DifferenceX);
        }
      }
    }
    else if(driverstationColor == DriverStation.Alliance.Red){
      for(int i = 4; i <= 7; i++){
        if(AprilTagPoses.apriltagPoses.get(i).getY() >= 0){
          DifferenceX = Drivetrain.getInstance().getPose().getX() - AprilTagPoses.apriltagPoses.get(i).getX();
          DifferenceY = Drivetrain.getInstance().getPose().getY() - AprilTagPoses.apriltagPoses.get(i).getY();

          angle = Math.atan(DifferenceY/DifferenceX);
        }
        else{
          DifferenceX = Drivetrain.getInstance().getPose().getX() + AprilTagPoses.apriltagPoses.get(i).getX();
          DifferenceY = Drivetrain.getInstance().getPose().getY() + AprilTagPoses.apriltagPoses.get(i).getY();

          angle = Math.atan(DifferenceY/DifferenceX);
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setVelocity(0.0);
    limelight.pauseLimelight();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
