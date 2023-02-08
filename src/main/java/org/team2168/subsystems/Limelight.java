// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private static Limelight instance = null;
  private NetworkTable networkTable;
  // standard entries

  private static boolean isLimelightEnabled;
  private static NetworkTableEntry tv;
  private static NetworkTableEntry tx;
  private static NetworkTableEntry ty;
  private static NetworkTableEntry ta;
  private static NetworkTableEntry tcornxy;
  private static double[] contourEntries = new double[4];
  private static NetworkTableEntry tl;
  private static NetworkTableEntry tc;
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
  // ledmode enum
  private static NetworkTableEntry ledMode;

  public enum LEDMode {
    CURRENTPIPELINE(0),
    FORCEOFF(1),
    FORCEBLINK(2),
    FORCEON(3);

    public final int value;

    private LEDMode(int value) {
      this.value = value;
    }
  }

  // cam mode
  private static NetworkTableEntry camMode;

  public enum CamMode {
    VISION_PROCESSOR(0),
    DRIVER_CAMERA(1);

    public final int camValue;

    private CamMode(int camValue) {
      this.camValue = camValue;
    }
  }

  // pipeline
  private static NetworkTableEntry pipeline;

  public enum Pipeline {
    PIPELINE_ZERO(0),
    PIPELINE_ONE(1),
    PIPELINE_TWO(2),
    PIPELINE_THREE(3),
    PIPELINE_FOUR(4),
    PIPELINE_FIVE(5),
    PIPELINE_SIX(6),
    PIPELINE_SEVEN(7),
    PIPELINE_EIGHT(8),
    PIPELINE_NINE(9);

    public final int pipelineValue;

    private Pipeline(int pipelineValue) {
      this.pipelineValue = pipelineValue;
    }
  }

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

  public enum Snapshot {
    RESET_SNAPSHOT_MODE(0),
    TAKE_ONE(1);

    public final int snapshotValue;

    private Snapshot(int snapshotValue) {
      this.snapshotValue = snapshotValue;
    }
  }

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

  public boolean hasTarget() {
    return tv.getDouble(0.0) == 1.0;
  }

  public double getOffsetX() {
    return tx.getDouble(0.0);
  }

  public double getOffsetY() {
    return ty.getDouble(0.0);
  }

  public double getTargetArea() {
    return ta.getDouble(0.0);
  }

  public void enableBaseCameraSettings() {
    camMode.setNumber(0);
    ledMode.setNumber(0);
    isLimelightEnabled = true;
  }

  public void setStreamMode(int streamValue) {
    stream.setNumber(streamValue);
  }

  public void enableVision(boolean turnOn) {

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

  public void takeSnapshot(int snapshotValue){
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

  public int getCurrentPipeline() {
    return getPipe.getNumber(0.0).intValue();
  }

  public double[] getBotPoseTranslation() {
    double[] botPoseArray = new double[6];
    return botPose.getDoubleArray(botPoseArray);
  }

  public double[] getCameraViewTranslation() {
    double[] cameraViewArray = new double[6];
    return cameraPoseTargetSpace.getDoubleArray(cameraViewArray);
  }

  public double[] getTargetPoseTranslation() {
    double[] targetPoseArray = new double[6];
    return targetPoseCameraSpace.getDoubleArray(targetPoseArray);
  }

  public double[] getTargetPoseInRobotSpace() {
    double[] targetPoseRobotSpaceArray = new double[6];
    return targetPoseRobotSpace.getDoubleArray(targetPoseRobotSpaceArray);
  }

  public double[] getBotPoseInTargetSpace() {
    double[] botPoseTargetSpaceArray = new double[6];
    return botPoseTargetSpace.getDoubleArray(botPoseTargetSpaceArray);
  }

  public void getDistanceMeters() {

  }

  public void getCropValues() {
    cropValues[0] = -1.0;
    cropValues[1] = 1.0;
    cropValues[2] = -1.0;
    cropValues[3] = 1.0;
    crop.setDoubleArray(cropValues);
  }

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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run\

    if (!isLimelightEnabled) {
      pauseLimelight();
    } else {

      enableBaseCameraSettings();
    }

  }
}