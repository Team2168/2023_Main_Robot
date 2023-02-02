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
 //standard entries
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
 //apritag entries

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

  public enum 

  public static Limelight getInstance(){
    if(instance == null){
      instance = new Limelight();
    }
    return instance;
  }

  public double getOffsetX(){
    return tx.getDouble(0.0);
  }

  public double getOffsetY(){
    return ty.getDouble(0.0);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
