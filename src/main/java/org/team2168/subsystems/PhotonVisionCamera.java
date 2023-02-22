// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.team2168.Constants;
import org.team2168.Constants.VisionConstants;

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
    // photonCamera = new PhotonCamera();
    photonCamera = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);

    try {
      // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
      AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      // creates photonPoseEstimator
      photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, photonCamera, VisionConstants.robotToCam);
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    } catch (IOException e) {
      // If AprilTagFieldLayout failed to load. We can't estimate poses if we don't know where the tags are.
      DriverStation.reportError("april tag filed layout failed to load", e.getStackTrace());
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
