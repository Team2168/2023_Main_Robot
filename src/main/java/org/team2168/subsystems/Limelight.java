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
  private NetworkTableEntry tv;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry tl;
  private NetworkTableEntry tc;
  private NetworkTableEntry tshort;
  private NetworkTableEntry tlong;
  private NetworkTableEntry thor;
  private NetworkTableEntry tvert;
  private NetworkTableEntry getPipe;
  private int x = 2;
  // apritag entries
  private NetworkTableEntry botPose;
  private NetworkTableEntry bitPoseWPIBlue;
  private NetworkTableEntry bosPoseWPIRed;
  private NetworkTableEntry cameraPoseTargetSpace;
  private NetworkTableEntry targetPoseCameraSpace;
  private NetworkTableEntry targetPoseRobotSpace;
  private NetworkTableEntry botPoseTargetSpace;
  private NetworkTableEntry tid;
  // ledmode enum
  private NetworkTableEntry ledMode;

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
  private NetworkTableEntry camMode;

  public enum CamMode {
    VISION_PROCESSOR(0),
    DRIVER_CAMERA(1);

    public final int camValue;

    private CamMode(int camValue) {
      this.camValue = camValue;
    }

    // pipeline
    private NetworkTableEntry pipeline;

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
  }
  //camera stream
  private NetworkTableEntry stream;
  public enum Stream {
    STANDARD(0),
    PIP_MAIN(1),
    PIP_SECONDARY(2);

    public final int streamValue;
    private Stream(int streamValue){
      this.streamValue = streamValue;
    }
//snapshot
private NetworkTableEntry snapshot;

public enum Snapshot {
  RESET_SNAPSHOT_MODE(0),
  TAKE_ONE(1);
  
  public final int snapshotValue;
  private Snapshot(int snapshotValue){
    this.snapshotValue = snapshotValue;
  }
}

  }

  public Limelight() {
    networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    tv = networkTable.getEntry("tv");
    tx = networkTable.getEntry("tx");
    ty = networkTable.getEntry("ty");
    ta = networkTable.getEntry("ta");
    tl = networkTable.getEntry("tl");
    tc = networkTable.getEntry("tc");
    tshort = networkTable.getEntry("tshort");
    tlong = networkTable.getEntry("tlong");
    thor = networkTable.getEntry("thor");
    tvert = networkTable.getEntry("tvert");
    getPipe = networkTable.getEntry("getpipe");
  }

  public static Limelight getInstance() {
    if (instance == null) {
      instance = new Limelight();
    }
    return instance;
  }

  public double getOffsetX() {
    return tx.getDouble(0.0);
  }

  public double getOffsetY() {
    return ty.getDouble(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
