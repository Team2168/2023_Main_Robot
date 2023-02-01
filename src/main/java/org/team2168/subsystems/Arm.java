// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;
import org.team2168.utils.TalonFXHelper;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private TalonFXHelper front_motor;
  private TalonFXHelper back_motor;

  // Current limit configuration
  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private static final boolean ENABLE_CURRENT_LIMIT = true;
  private static final double CONTINUOUS_CURRENT_LIMIT = 20; // amps TODO: update limit
  private static final double TRIGGER_THRESHOLD_LIMIT = 30; // amp TODO: update limit
  private static final double TRIGGER_THRESHOLD_TIME = 0.2; // s 
  private static final double NEUTRAL_DEADBAND = 0.001;

  private static final double TICKS_PER_REV = 2048;
  private static final double GEAR_RATIO = 0.0; //TODO: update value, teeth/diameter

  private static final double kPeakOutput = 1.0;
  private static final int kPIDLoopIdx = 0;
  private static final int kTimeoutMs = 30;
  private static boolean kSensorPhase = false;
  private static TalonFXInvertType kMotorInvert = TalonFXInvertType.Clockwise;

  private static final double ACCELERATION_LIMIT = 0.0; //TODO: update value after testing
  private static final double CRUISE_VELOCITY_LIMIT = 0.0; //TODO: update value after testing
 
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
    front_motor.setNeutralMode(NeutralMode.Brake);

    front_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
    front_motor.setSensorPhase(kSensorPhase);
    front_motor.setInverted(kMotorInvert);

    front_motor.configNominalOutputForward(0, kTimeoutMs);
    front_motor.configNominalOutputReverse(0, kTimeoutMs);
    front_motor.configPeakOutputForward(kPeakOutput, kTimeoutMs);
    front_motor.configPeakOutputReverse(-kPeakOutput, kTimeoutMs);

    front_motor.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
    front_motor.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
    front_motor.config_kD(kPIDLoopIdx, kD, kTimeoutMs);
    front_motor.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
    front_motor.configMotionAcceleration(ACCELERATION_LIMIT);
    front_motor.configMotionCruiseVelocity(CRUISE_VELOCITY_LIMIT);
    front_motor.configAllowableClosedloopError(0, ACCELERATION_LIMIT, kTimeoutMs);
    
    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CONTINUOUS_CURRENT_LIMIT, 
      TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);
    
    front_motor.configSupplyCurrentLimit(talonCurrentLimit);

    front_motor.configClosedLoopStatusFrameRates();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
