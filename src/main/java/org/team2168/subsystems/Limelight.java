// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Limelight extends SubsystemBase implements Loggable {
  private static Limelight instance = null;
  private NetworkTable networkTable;

  public double[] botPoseArray = new double[6];
  public double[] targetPoseArray = new double[6];
  public double[] targetPoseRobotSpaceArray = new double[6];
  public double[] botPoseTargetSpaceArray = new double[6];
  public double[] cameraViewArray = new double[6];
  // standard entries

  private static boolean isLimelightEnabled;
  public Pose3d apriltagValue;
  private static NetworkTableEntry tv;
  private static NetworkTableEntry tx;
  private static NetworkTableEntry ty;
  private static NetworkTableEntry ta;
  private static NetworkTableEntry tcornxy;
  private static double[] contourEntries = new double[4];
  private static NetworkTableEntry tl;
  private static NetworkTableEntry tc;
  private static NetworkTableEntry cl;
  private static NetworkTableEntry tshort;
  private static NetworkTableEntry tlong;
  private static NetworkTableEntry thor;
  private static NetworkTableEntry tvert;
  private static NetworkTableEntry getPipe;
  // apritag entries
  private static NetworkTableEntry botPose;
  private static NetworkTableEntry botPoseWPIBlue;
  private static NetworkTableEntry botPoseWPIRed;
  private static NetworkTableEntry cameraPoseTargetSpace;
  private static NetworkTableEntry targetPoseCameraSpace;
  private static NetworkTableEntry targetPoseRobotSpace;
  private static NetworkTableEntry botPoseTargetSpace;
  private static NetworkTableEntry tid;
  private static NetworkTableEntry cameraPoseRobotSpace;
  // public static double camerapose_x;
  // public static double camerapose_y;
  // public static double camerapose_z;
  // public static double camerapose_roll;
  // public static double camerapose_pitch;
  // public static double camerapose_yaw;
  // public static double apriltagPose_x;
  // public static double apriltagPose_y;
  // public static double apriltagPose_z;
  // public static double apriltagPose_roll;
  // public static double apriltagPose_Pitch;
  // public static double apriltagPose_Yaw;
  // public static double botPoseRelativeToTag_x;
  // public static double botPoseRelativeToTag_y;
  // public static double botPoseRelativeToTag_z;
  // public static double botPoseRelativeToTag_roll;
  // public static double botPoseRelativeToTag_pitch;
  // public static double botPoseRelativeToTag_yaw;

  // ledmode enum
  private static NetworkTableEntry ledMode;

  // public enum LEDMode {
  // CURRENTPIPELINE(0),
  // FORCEOFF(1),
  // FORCEBLINK(2),
  // FORCEON(3);

  // public final int value;

  // private LEDMode(int value) {
  // this.value = value;
  // }
  // }

  // cam mode
  private static NetworkTableEntry camMode;

  // public enum CamMode {
  // VISION_PROCESSOR(0),
  // DRIVER_CAMERA(1);

  // public final int camValue;

  // private CamMode(int camValue) {
  // this.camValue = camValue;
  // }
  // }

  // pipeline
  private static NetworkTableEntry pipeline;

  // public enum Pipeline {
  // PIPELINE_ZERO(0),
  // PIPELINE_ONE(1),
  // PIPELINE_TWO(2),
  // PIPELINE_THREE(3),
  // PIPELINE_FOUR(4),
  // PIPELINE_FIVE(5),
  // PIPELINE_SIX(6),
  // PIPELINE_SEVEN(7),
  // PIPELINE_EIGHT(8),
  // PIPELINE_NINE(9);

  // public final int pipelineValue;

  // private Pipeline(int pipelineValue) {
  // this.pipelineValue = pipelineValue;
  // }
  // }

  // camera stream
  private static NetworkTableEntry stream;

  public enum Stream {
    STANDARD(0),
    PIP_MAIN(1),
    PIP_SECONDARY(2);

    public final int streamValue;

    private Stream(int streamValue) {
      this.streamValue = streamValue;
    }
  }

  // snapshot
  private static NetworkTableEntry snapshot;

  // public enum Snapshot {
  // RESET_SNAPSHOT_MODE(0),
  // TAKE_ONE(1);

  // public final int snapshotValue;

  // private Snapshot(int snapshotValue) {
  // this.snapshotValue = snapshotValue;
  // }
  // }

  // crop values
  private static NetworkTableEntry crop;
  private double[] cropValues = new double[4];

  public Limelight() {
    networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    init();
    isLimelightEnabled = false;
  }

  public static Limelight getInstance() {
    if (instance == null) {
      instance = new Limelight();
    }
    return instance;
  }

  @Config(name = "hasTarget:")
  public boolean hasTarget() {
    return tv.getDouble(0.0) == 1.0;
  }

  @Log(name = "Horizontal Angle: ", rowIndex = 2, columnIndex = 3)
  public double getOffsetX() {
    return tx.getDouble(0.0);
  }

  @Log(name = "Vertical Angle", rowIndex = 3, columnIndex = 3)
  public double getOffsetY() {
    return ty.getDouble(0.0);
  }

  public double getTargetArea() {
    return ta.getDouble(0.0);
  }

  public void enableBaseCameraSettings() {
    setCamMode(0);
    setLedMode(0);
    setPipeline(1);
    enableVision(true);
    isLimelightEnabled = true;
  }

  public void setStreamMode(int streamValue) {
    stream.setNumber(streamValue);
  }

  public void enableVision(boolean turnOn) {

    isLimelightEnabled = true;
    init();
  }

  public void setCamMode(int camValue) {
    camMode.setNumber(camValue);
  }

  public void setLedMode(int ledValue) {
    ledMode.setNumber(ledValue);
  }

  public void setPipeline(int pipelineValue) {
    pipeline.setNumber(pipelineValue);
  }

  public void takeSnapshot(int snapshotValue) {
    snapshot.setNumber(snapshotValue);
  }

  public void pauseLimelight() {
    setCamMode(1);
    setLedMode(1);
    setPipeline(0);
    isLimelightEnabled = false;

  }

  public boolean isLimelightEnabled() {
    return isLimelightEnabled;
  }

  @Log(name = "Current Pipeline: ", rowIndex = 4, columnIndex = 3)
  public int getCurrentPipeline() {
    return getPipe.getNumber(0.0).intValue();
  }

  public double getLatencyMs() {
    return tl.getDouble(0.0);
  }

  public double getCapturedLatencyTime() {
    return cl.getDouble(0.0);
  }

  // deal with this problem of null pointer exception.
  public double[] getBotPoseTranslation() {
    // double[] botPoseArray = new double[6];
    botPose.getDoubleArray(botPoseArray);
    return botPoseArray;
  }

  // public double[] botPoseArrayTwo = new double[2];

  public double[] getCameraViewTranslation() {

    // double[] cameraViewArray = new double[6];

    cameraPoseTargetSpace.getDoubleArray(cameraViewArray);
    return cameraViewArray;
  }

  public double[] getTargetPoseTranslation() {
    // double[] targetPoseArray = new double[6];
    targetPoseCameraSpace.getDoubleArray(targetPoseArray);
    return targetPoseArray;
  }

  public double[] getTargetPoseInRobotSpace() {
    // double[] targetPoseRobotSpaceArray = new double[6];
    targetPoseRobotSpace.getDoubleArray(targetPoseRobotSpaceArray);
    return targetPoseRobotSpaceArray;
  }

  public double[] getBotPoseInTargetSpace() {
    // double[] botPoseTargetSpaceArray = new double[6];
    botPoseTargetSpace.getDoubleArray(botPoseTargetSpaceArray);
    return botPoseTargetSpaceArray;
  }

  public Pose3d getPoseInTargetSpace() {
    return new Pose3d(
        new Translation3d(botPoseTargetSpaceArray[0], botPoseTargetSpaceArray[1], botPoseTargetSpaceArray[2]),
        new Rotation3d(botPoseTargetSpaceArray[3], botPoseTargetSpaceArray[4], botPoseTargetSpaceArray[5]));
  }

  @Log(name = "Current ApriltagID")
  public double getAprilTagID() {
    return tid.getDouble(0.0);
  }

  // empty until current appropiate robot data is avaliable to make this method
  // work.
  // public void getDistanceMeters() {

  // }

  public void getCropValues() {
    cropValues[0] = -1.0;
    cropValues[1] = 1.0;
    cropValues[2] = -1.0;
    cropValues[3] = 1.0;
    crop.setDoubleArray(cropValues);
  }

  @Log(name = "Is Connection Established: ", rowIndex = 4, columnIndex = 4)
  public boolean isConnectionEstablished() {
    if (!(tx == null)) {
      return true;
    } else {
      return false;
    }

  }

  public double[] getRawContourCornerData() {
    return tcornxy.getDoubleArray(contourEntries);
  }

  @Log(name = "Average Contour Data: ", rowIndex = 5, columnIndex = 4)
  public double getAvgContourCornerData() {
    double average;
    getRawContourCornerData();
    average = ((contourEntries[0] + contourEntries[1] + contourEntries[2] + contourEntries[3]) /
        contourEntries.length);
    return average;
  }

  private void init() {
    tv = networkTable.getEntry("tv");
    tx = networkTable.getEntry("tx");
    ty = networkTable.getEntry("ty");
    ta = networkTable.getEntry("ta");
    tcornxy = networkTable.getEntry("tcornxy");
    tl = networkTable.getEntry("tl");
    tc = networkTable.getEntry("tc");
    tshort = networkTable.getEntry("tshort");
    tlong = networkTable.getEntry("tlong");
    thor = networkTable.getEntry("thor");
    tvert = networkTable.getEntry("tvert");
    getPipe = networkTable.getEntry("getpipe");
    ledMode = networkTable.getEntry("ledMode");
    camMode = networkTable.getEntry("camMode");
    pipeline = networkTable.getEntry("pipeline");
    stream = networkTable.getEntry("stream");
    snapshot = networkTable.getEntry("snapshot");
    crop = networkTable.getEntry("crop");
    botPose = networkTable.getEntry("botpose");
    botPoseWPIRed = networkTable.getEntry("botpose_wpired");
    botPoseWPIBlue = networkTable.getEntry("botpose_wpiblue");
    cameraPoseTargetSpace = networkTable.getEntry("camerapose_targetspace");
    targetPoseCameraSpace = networkTable.getEntry("targetpose_cameraspace");
    targetPoseRobotSpace = networkTable.getEntry("targetpose_robotspace");
    botPoseTargetSpace = networkTable.getEntry("botpose_targetspace");
    tid = networkTable.getEntry("tid");
    cl = networkTable.getEntry("cl");
    cameraPoseRobotSpace = networkTable.getEntry("camerapose_robotspace");
  }

  public Pose3d getPose3d() {
    return new Pose3d(botPoseArray[0], botPoseArray[1], botPoseArray[2],
        new Rotation3d(Units.degreesToRadians(botPoseArray[3]), Units.degreesToRadians(botPoseArray[4]),
            Units.degreesToRadians(botPoseArray[5])));

  }

  public Pose2d getPose2d() {
    return new Pose2d(botPoseArray[0], botPoseArray[2],
        new Rotation2d(Units.degreesToRadians(botPoseArray[5])));
  }

  public Pose3d getCameraTransformFromBotPose() {
    return getPose3d().transformBy(new Transform3d(new Translation3d(cameraViewArray[0], cameraViewArray[1],
        cameraViewArray[2]),
        new Rotation3d(Units.degreesToRadians(cameraViewArray[3]),
            Units.degreesToRadians(cameraViewArray[4]), Units.degreesToRadians(cameraViewArray[5]))));
  }

  public Pose3d getTagTransformFromBotPose() {
    return getPose3d().transformBy(new Transform3d(getPose3d(), getAprilTagPoseRelativeToLimelight()));
  }

  public Pose3d getTagTransformFromBotPoseReal() {
    return getPose3d().transformBy(new Transform3d(getPose3d(), getApriltagDimensionsFromFidicualId()));
  }

  public Pose3d getAprilTagPoseRelativeToLimelight() {
    return new Pose3d(new Translation3d(targetPoseArray[0], targetPoseArray[1], targetPoseArray[2]),
        new Rotation3d(targetPoseArray[3], targetPoseArray[4], targetPoseArray[5]));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run\

    getBotPoseTranslation();
    getBotPoseInTargetSpace();
    getCameraViewTranslation();
    getTargetPoseTranslation();
    getTargetPoseInRobotSpace();
    getBotPoseInTargetSpace();

    if (!isLimelightEnabled) {
      pauseLimelight();
    }
    setPipeline((int) pipeline.getDouble(0.0));

    var apriltagId = (int) Math.round(getAprilTagID());

    switch (apriltagId) {
      case 1:
        apriltagValue = Constants.AprilTagPoses.apriltagPoses.get(0);
        break;
      case 2:
        apriltagValue = Constants.AprilTagPoses.apriltagPoses.get(1);
        break;
      case 3:
        apriltagValue = Constants.AprilTagPoses.apriltagPoses.get(2);
        break;
      case 4:
        apriltagValue = Constants.AprilTagPoses.apriltagPoses.get(3);
        break;
      case 5:
        apriltagValue = Constants.AprilTagPoses.apriltagPoses.get(4);
        break;
      case 6:
        apriltagValue = Constants.AprilTagPoses.apriltagPoses.get(5);
        break;
      case 7:
        apriltagValue = Constants.AprilTagPoses.apriltagPoses.get(6);
        break;
      case 8:
        apriltagValue = Constants.AprilTagPoses.apriltagPoses.get(7);
        break;
      default:
        apriltagValue = new Pose3d();

    }

  }

  public double getEstimatedDistanceFromVision() {
    return Math.sqrt((Math.pow(this.getAprilTagPoseRelativeToLimelight().getZ() - this.getPose2d().getY(), 2)
        + Math.pow(this.getAprilTagPoseRelativeToLimelight().getX() - this.getPose2d().getX(), 2)));
  }

  public double getEstimatedDistanceFromRealApriltagDimensions() {
    return Math.sqrt((Math.pow(this.getApriltagDimensionsFromFidicualId().getZ() - this.getPose2d().getY(), 2)
        + Math.pow(this.getApriltagDimensionsFromFidicualId().getX() - this.getPose2d().getX(), 2)));
  }

  public Pose3d getApriltagDimensionsFromFidicualId() {
    return apriltagValue;
  }

}
