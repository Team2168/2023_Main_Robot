// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;
import org.team2168.utils.TalonFXHelper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private static TalonFXHelper armMotor;

  private static double setpoint = 0.0;

  // Current limit configuration
  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private static final boolean ENABLE_CURRENT_LIMIT = true;
  private static final double CONTINUOUS_CURRENT_LIMIT = 20; // amps TODO: update limit
  private static final double TRIGGER_THRESHOLD_LIMIT = 30; // amp TODO: update limit
  private static final double TRIGGER_THRESHOLD_TIME = 0.2; // s 
  private static final double NEUTRAL_DEADBAND = 0.001;

  private static final double TICKS_PER_REV = 2048;
  private static final double GEAR_RATIO = 0.0; //TODO: update value, teeth/diameter
  private static final double TICKS_PER_ARM_ROTATION = TICKS_PER_REV * GEAR_RATIO;

  private static final double TICKS_PER_SECOND = TICKS_PER_REV;
  private static final double TICKS_PER_100_MS = TICKS_PER_REV/ 10.0;
  private static final double ONE_HUNDRED_MS_PER_MINUTE = 1000.0/600000.0;

  private static final double ARM_MIN_ROTATION_TICKS = -10000; 
  private static final double ARM_MAX_ROTATION_TICKS = 10000; //TODO: update number  

  private static final double ARM_MIN_ROTATION_DEGREES = ticksToDegrees(ARM_MIN_ROTATION_TICKS);
  private static final double ARM_MAX_ROTATION_DEGREES = ticksToDegrees(ARM_MAX_ROTATION_TICKS);
  private static final double ARM_TOTAL_DEGREES = Math.abs(ARM_MAX_ROTATION_DEGREES) + Math.abs(ARM_MIN_ROTATION_DEGREES);

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
    armMotor = new TalonFXHelper(Constants.ARM_MOTOR);

    //arm config
    armMotor.configFactoryDefault();
    armMotor.configNeutralDeadband(NEUTRAL_DEADBAND);
    armMotor.setNeutralMode(NeutralMode.Brake);

    armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
    armMotor.setSensorPhase(kSensorPhase);
    armMotor.setInverted(kMotorInvert);

    armMotor.configNominalOutputForward(0, kTimeoutMs);
    armMotor.configNominalOutputReverse(0, kTimeoutMs);
    armMotor.configPeakOutputForward(kPeakOutput, kTimeoutMs);
    armMotor.configPeakOutputReverse(-kPeakOutput, kTimeoutMs);

    armMotor.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
    armMotor.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
    armMotor.config_kD(kPIDLoopIdx, kD, kTimeoutMs);
    armMotor.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
    armMotor.configMotionAcceleration(ACCELERATION_LIMIT);
    armMotor.configMotionCruiseVelocity(CRUISE_VELOCITY_LIMIT);
    armMotor.configAllowableClosedloopError(0, ACCELERATION_LIMIT, kTimeoutMs);
    
    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CONTINUOUS_CURRENT_LIMIT, 
      TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);
    
    armMotor.configSupplyCurrentLimit(talonCurrentLimit);

    armMotor.configClosedLoopStatusFrameRates();
  }

  private static double ticksToDegrees(double ticks) {
    return (ticks/TICKS_PER_ARM_ROTATION)*360;
  }

  private static double degreesToTicks(double degrees) {
    return (degrees/360)*TICKS_PER_ARM_ROTATION;
  }

  private static double ticksPer100msToDegreesPerSecond(double ticks) {
    return ticksToDegrees(ticks) * 10.0;
  }

  public static double degreesPerSecondToTicksPer100ms(double degrees) {
    return degreesToTicks(degrees) / 10.0;
  }

  public static void setArmRotationDegrees(double degrees) {
    var demand = MathUtil.clamp(degrees, ARM_MIN_ROTATION_DEGREES, ARM_MAX_ROTATION_DEGREES);
    setpoint = degrees;
    armMotor.set(ControlMode.MotionMagic, degreesToTicks(demand));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}