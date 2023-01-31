// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;
import org.team2168.utils.TalonFXHelper;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private TalonFXHelper front_motor;
  private TalonFXHelper back_motor;

  // Current limit configuration
  private static final boolean ENABLE_CURRENT_LIMIT = true;
  private static final double CONTINUOUS_CURRENT_LIMIT = 20; // amps TODO: update limit
  private static final double TRIGGER_THRESHOLD_LIMIT = 30; // amp TODO: update limit
  private static final double TRIGGER_THRESHOLD_TIME = 0.2; // s 
  private static final double NEUTRAL_DEADBAND = 0.001;

  private static final double TICKS_PER_REV = 2048;
  private static final double GEAR_RATIO = 0.0; //TODO: update value, teeth/diameter

  private static final int kPIDLoopIdx = 0;
  private static final int kTimeoutMs = 30;
  private static boolean kSensorPhase = false;
  private static TalonFXInvertType kMotorInvert = TalonFXInvertType.Clockwise;
 
  private static Arm instance = null;

  public Arm getInstance() {
    if (instance == null)
      instance = new Arm();
    return instance;  
  }    

  // Gains
  private static final double kP;
  private static final double kI;
  private static final double kD;
  private static final double kF;

  static{
    kP = 0.0;
    kI = 0.0;
    kD = 0.0;
    kF = 0.0;
  }
  

  private Arm() {
    front_motor = new TalonFXHelper(Constants.ARM_FRONT_MOTOR);
    back_motor = new TalonFXHelper(Constants.ARM_BACK_MOTOR);

    front_motor.configFactoryDefault();
    front_motor.configNeutralDeadband(NEUTRAL_DEADBAND);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
