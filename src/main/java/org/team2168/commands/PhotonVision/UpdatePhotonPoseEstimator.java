// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.PhotonVision;

import org.team2168.subsystems.PhotonVisionCamera;
import org.team2168.subsystems.Turret;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class UpdatePhotonPoseEstimator extends CommandBase {
  /** Creates a new UpdatePhotonPoseEstimator. */
  PhotonVisionCamera photonVisionCamera;
  Turret turret;
  Transform3d robotToCam;
  double turretRadius = Units.inchesToMeters(7.115);
  public UpdatePhotonPoseEstimator(PhotonVisionCamera photonVisionCamera, Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.photonVisionCamera = photonVisionCamera;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotToCam = new Transform3d(new Translation3d(-turretRadius*Math.sin(turret.getTurretAngle()), turretRadius*Math.cos(turret.getTurretAngle()), 0.0), new Rotation3d(0.0, 0.0, 0.0));
    photonVisionCamera.photonPoseEstimator.setRobotToCameraTransform(robotToCam);
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
