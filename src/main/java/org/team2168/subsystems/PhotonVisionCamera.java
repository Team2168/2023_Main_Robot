// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class PhotonVisionCamera extends SubsystemBase {
  private PhotonCamera photonCamera;
  private PhotonPoseEstimator photonPoseEstimator;
  /** Creates a new PhotonVision. */

  private static PhotonVisionCamera instance;
  private static NetworkTable networkTable;
  private static NetworkTableEntry latencyMillis;
  private static NetworkTableEntry targetPitch;
  private static NetworkTableEntry hasTarget;
  private static NetworkTableEntry targetYaw;
  private static NetworkTableEntry targetArea;
  private static NetworkTableEntry targetSkew;
  private static NetworkTableEntry targetPose;

  public PhotonVisionCamera() {
    networkTable = NetworkTableInstance.getDefault().getTable("photonvision");

    try {
      AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch (IOException e) {
      
    }
  }

  public static PhotonVisionCamera getInstance() {
    if (instance == null) {
      instance = new PhotonVisionCamera();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
