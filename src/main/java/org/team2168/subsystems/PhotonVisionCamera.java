// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// adapted from Photonvision's AprilTag example code.
package org.team2168.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team2168.Constants;
import org.team2168.Constants.VisionConstants;

public class PhotonVisionCamera extends SubsystemBase {
  public PhotonCamera photonCamera;
  public PhotonPoseEstimator photonPoseEstimator;
  private Transform3d robotToCam;
  /** Creates a new PhotonVision. */

  private static AprilTagFieldLayout fieldLayout;

  private static PhotonVisionCamera instance;
  private static NetworkTableInstance networkTableInstance;
  private static PhotonPipelineResult result;
  private static List<PhotonTrackedTarget> targets;
  private static PhotonTrackedTarget singleTarget;

  private static double xCalcCameraPose;
  private static double yCalcCameraPose;

  private static double sumOfXCalcCameraPose;
  private static double sumOfYCalcCameraPose;

  private static double avgXCameraPose;
  private static double avgYCameraPose;

  private static Pose3d avgCameraPose;
  private static int targetID;
  private static Pose3d targetPose;

  public PhotonVisionCamera() {
    // photonCamera = new PhotonCamera();
    networkTableInstance = NetworkTableInstance.getDefault();
    photonCamera = new PhotonCamera(networkTableInstance, Constants.VisionConstants.CAMERA_NAME);
    robotToCam = new Transform3d(new Translation3d(7.115, 0.0, 0.5), // change for real distance between photonvision cam and center of robot.
    new Rotation3d(0, 0, 0));

    result = new PhotonPipelineResult();

    try {
      // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
      AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      // creates photonPoseEstimator
      photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, photonCamera, VisionConstants.robotToCam);
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    } catch (IOException e) {
      // If AprilTagFieldLayout failed to load. We can't estimate poses if we don't know where the tags are.
      DriverStation.reportError("april tag filed layout failed to load", e.getStackTrace());
      photonPoseEstimator = null;
    }
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (photonPoseEstimator == null) {
      return Optional.empty();
    }
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
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
    sumOfXCalcCameraPose = 0.0;
    sumOfYCalcCameraPose = 0.0;

    result = photonCamera.getLatestResult();
    targets = result.getTargets();

    for (int i = 0; i < targets.size(); i++){
      singleTarget = targets.get(i);
      targetID = singleTarget.getFiducialId();
      targetPose = fieldLayout.getTagPose(targetID).get();
      
      sumOfXCalcCameraPose = sumOfXCalcCameraPose + (targetPose.getX() - singleTarget.getBestCameraToTarget().getX());
      sumOfYCalcCameraPose = sumOfYCalcCameraPose + (targetPose.getY() - singleTarget.getBestCameraToTarget().getY());
    }

    avgXCameraPose = sumOfXCalcCameraPose/targets.size();
    avgYCameraPose = sumOfYCalcCameraPose/targets.size();
    avgCameraPose = new Pose3d(avgXCameraPose, avgYCameraPose, Constants.VisionConstants.CAMERA_HEIGHT_M, new Rotation3d()); // replace Rotation3d with Drivetrain gyro angle
  }
}