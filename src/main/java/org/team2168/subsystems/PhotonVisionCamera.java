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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
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

  private static double latencySeconds;

  private static double xCalcCameraPose;
  private static double yCalcCameraPose;

  private static double sumCalcRobotPoseX;
  private static double sumCalcRobotPoseY;
  private static double sumCalcRobotPoseZ;

  private static double sumOfXCalcCameraPose;
  private static double sumOfYCalcCameraPose;

  private static double avgXCameraPose;
  private static double avgYCameraPose;

  private static double calcRobotPoseRadians;

  private static Pose3d avgCameraPose;
  private static Pose3d avgRobotPose;
  private static int targetID;
  private static Pose3d targetPose;
  private static Pose3d singleTagRobotPose;
  private static double numTargets;

  private static double imageCaptureTime;

  public PhotonVisionCamera() {
    // photonCamera = new PhotonCamera();
    networkTableInstance = NetworkTableInstance.getDefault();
    photonCamera = new PhotonCamera(networkTableInstance, Constants.VisionConstants.CAMERA_NAME);
    robotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(7.115), 0.0, 0.5), // change for real distance between photonvision cam and center of robot.
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

  /**
   * Updates estimated robot pose based on whatever it is given as the previous pose.
   * @param prevEstimatedRobotPose last robot pose estimated by the pose estimator
   * @return the updated robot pose
   */
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
  /**
   * gets an average Robot Pose compiled from a sum of data from different Targets
   * @return the calculated average Robot Pose relative to the field
   */
  public Pose3d getAvgRobotPose() {
    return avgRobotPose;
  }

  /**
   * gets a list of all targets tracked by the photonvision camera at one time
   * @return a list of tracked targets
   */
  public List<PhotonTrackedTarget> getTargets() {
    return photonCamera.getLatestResult().getTargets();
  }

  /**
   * gets best target out of the list of targets detected. "best" is evaluated
   * based on the target sort - which is the most prioritized by the target sort
   * mode?
   * @return the best target from all targets detected
   */
  public PhotonTrackedTarget getBestTarget() {
    return photonCamera.getLatestResult().getBestTarget();
  }

  /**
   * gets the AprilTagFieldLayout, storing information such as tag pose and origin position
   * of the field.
   * @return the field layout for this year's game
   */
  public AprilTagFieldLayout getFieldLayout() {
    return fieldLayout;
  }

  /**
   * gets latency of the current pipeline result
   * @return the latency of the result in seconds
   */
  public double getLatencySecs() {
    return latencySeconds;
  }

  /**
   * gets the condition for if the result detects a target or not
   * @return true if target detected, false if not detected.
   */
  public boolean getHasTarget() {
    return result.hasTargets();
  }

  /**
   * gets pitch (rotation around the side-facing axis) of the target, relative to the
   * center of view of the result.
   * @return pitch of the target relative to the crosshair. positive means target is
   * above the cross hair, negative means target is below.
   */
  public double getTargetPitch() {
    return result.getBestTarget().getPitch();
  }

  /**
   * gets yaw (rotation around the vertical axis) of the target, relative to the
   * center of view of the result.
   * @return yaw of the target relative to the crosshair. positive means the target is right
   * of cross hair, negative means target is left of the crosshair.
   */
  public double getTargetYaw() {
    return result.getBestTarget().getYaw();
  }

  /**
   * gets how much of the camera feed the bounding box takes up.
   * @return percentage of camera feed bounding box takes up.
   */
  public double getTargetArea() {
    return result.getBestTarget().getArea();
  }

  /**
   * gets what angle the target is facing relative to the camera's point of view
   * @return angle target is facing (counterclockwise to directly facing camera is positive)
   */
  public double getTargetSkew() {
    return result.getBestTarget().getSkew();
  }

  /**
   * gets a double array from which all data was thrown into.
   * @return double array with pose data in order of: x position, y position, z position,
   * and a quaternion's w, x, y, and z values.
   */
  public double[] getTargetPoseValues() {
    double[] defaultValue = new double[1];
    defaultValue[0] = 0.0;
    return networkTableInstance.getEntry("targetPose").getDoubleArray(defaultValue);
  }

  /**
   * gets amount of pixels of the camera feed the target horizontally spans across
   * @return pixels of target in the x-direction
   */
  public double getTargetPixelsX() {
    return networkTableInstance.getEntry("targetPixelsX").getDouble(0.0);
  }

  /**
   * gets amount of pixels of the camera feed the target vertically spans across
   * @return pixels of target in the y-direction
   */
  public double getTargetPixelsY() {
    return networkTableInstance.getEntry("targetPixelsY").getDouble(0.0);
  }

  /**
   * sets the current pipeline index
   * @param index the value to set the current pipeline to
   */
  public void setPipelineIndex(int index) {
    networkTableInstance.getEntry("pipelineIndex").setInteger(index);
  }

  /**
   * gets current pipeline index
   * @return the integer value of the pipeline index
   */
  public int getPipelineIndex() {
    return (int) networkTableInstance.getEntry("pipelineIndex").getInteger(0);
  }

  /**
   * sets the photonvision camera to either be in driver mode or not
   * @param isOn value to determine whether camera is in driver mode or not-
   * true indicates driver mode, false indicates normal target detection settings.
   */
  public void setDriverMode(boolean isOn) {
    networkTableInstance.getEntry("driverMode").setBoolean(isOn);
  }

  /**
   * gets the mode of the photonvision camera
   * @return if camera is in driver mode or not. true indicates being in driver mode,
   * false indicates not being in driver mode.
   */
  public boolean getDriverMode() {
    return networkTableInstance.getEntry("driverMode").getBoolean(false);
  }

  /**
   * sets the led mode of the photonvision
   * @param ledMode mode to set the photonvision to: (-1 = default, 0 is off, 1 is on, 2 is blink)
   */
  public void setLedMode(int ledMode) {
    networkTableInstance.getTable("photonvision").getEntry("ledMode").setInteger(ledMode);
  }

  /**
   * gets the current led mode of the photonvision
   * @return either -1, 0, 1, or 2 to represent modes default, off, on, and blink, in that order.
   */
  public int getLedMode() {
    return (int) networkTableInstance.getTable("photonvision").getEntry("ledMode").getInteger(-1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sumCalcRobotPoseX = 0.0;
    sumCalcRobotPoseY = 0.0;
    sumCalcRobotPoseZ = 0.0;

    result = photonCamera.getLatestResult();
    targets = result.getTargets();
    numTargets = targets.size();
    imageCaptureTime = result.getTimestampSeconds();
    if (result.hasTargets()) {
    for (int i = 0; i < numTargets; i++){
      singleTarget = targets.get(i);
      targetID = singleTarget.getFiducialId();
      targetPose = fieldLayout.getTagPose(targetID).get();

      singleTagRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(singleTarget.getBestCameraToTarget(), fieldLayout.getTagPose(singleTarget.getFiducialId()).get(), robotToCam);
      sumCalcRobotPoseX = sumCalcRobotPoseX + singleTagRobotPose.getX();
      sumCalcRobotPoseY = sumCalcRobotPoseY + singleTagRobotPose.getY();
      sumCalcRobotPoseZ = sumCalcRobotPoseZ + singleTagRobotPose.getZ();
      
      // sumOfXCalcCameraPose = sumOfXCalcCameraPose + (targetPose.getX() - singleTarget.getBestCameraToTarget().getX());
      // sumOfYCalcCameraPose = sumOfYCalcCameraPose + (targetPose.getY() - singleTarget.getBestCameraToTarget().getY());
    }

    latencySeconds = result.getLatencyMillis()/1000.0;
    avgRobotPose = new Pose3d(sumCalcRobotPoseX/numTargets, sumCalcRobotPoseY/numTargets, sumCalcRobotPoseZ/numTargets, new Rotation3d()); // rotation should use Drivetrain gyro Angle
  }

    // avgXCameraPose = sumOfXCalcCameraPose/targets.size();
    // avgYCameraPose = sumOfYCalcCameraPose/targets.size();
    // avgCameraPose = new Pose3d(avgXCameraPose, avgYCameraPose, Constants.VisionConstants.CAMERA_HEIGHT_M, new Rotation3d()); // replace Rotation3d with Drivetrain gyro angle
  }
}
