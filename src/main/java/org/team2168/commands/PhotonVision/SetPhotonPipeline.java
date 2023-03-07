// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.PhotonVision;

import org.photonvision.common.hardware.VisionLEDMode;
import org.team2168.subsystems.PhotonVisionCamera;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetPhotonPipeline extends CommandBase {
  /** Creates a new SetPhotonPipeline. */
  PhotonVisionCamera photonVisionCamera;
  int pipelineNum;
  boolean isPipelineSet = false;
  
  public SetPhotonPipeline(PhotonVisionCamera photonVisionCamera, int pipelineNum) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pipelineNum = pipelineNum;
    this.photonVisionCamera = photonVisionCamera;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isPipelineSet = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    photonVisionCamera.photonCamera.setPipelineIndex(pipelineNum);
    photonVisionCamera.photonCamera.setLED(VisionLEDMode.kDefault);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isPipelineSet;
  }
}
