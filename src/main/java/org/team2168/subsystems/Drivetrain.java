// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants.CANDevices;
import org.team2168.utils.PigeonHelper;
import org.team2168.utils.TalonFXHelper;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private PigeonHelper pidgey;

  private TalonFXHelper leftMotor1;
  private TalonFXHelper leftMotor2;
  private TalonFXHelper rightMotor1;
  private TalonFXHelper rightMotor2;

  private DifferentialDriveOdometry odometry;

  private static Drivetrain instance = null;

  private static final boolean ENABLE_CURRENT_LIMIT = true;
  private static final double CONTINUOUS_CURRENT_LIMIT = 35; // amps
  private static final double TRIGGER_THRESHOLD_LIMIT = 40; // amp
  private static final double TRIGGER_THRESHOLD_TIME = 0.2; // s
  private final static double NEUTRALDEADBAND = 0.001;

  private SupplyCurrentLimitConfiguration talonCurrentLimit;

  /**
   * Config Objects for motor controllers
  */
  TalonFXConfiguration leftConfig = new TalonFXConfiguration();
  TalonFXConfiguration rightConfig = new TalonFXConfiguration();
  
  /**
   * Invert Directions for Left and Right
   */
  TalonFXInvertType leftInvert = TalonFXInvertType.CounterClockwise; // Same as invert = "false"
  TalonFXInvertType rightInvert = TalonFXInvertType.Clockwise; // Same as invert = "true"

  
  public Drivetrain() {
 // Instantiate motor objects
  leftMotor1 = new TalonFXHelper(CANDevices.DRIVETRAIN_LEFT_MOTOR_1);
  leftMotor2 = new TalonFXHelper(CANDevices.DRIVETRAIN_LEFT_MOTOR_2);
  rightMotor1 = new TalonFXHelper(CANDevices.DRIVETRAIN_RIGHT_MOTOR_1);
  rightMotor2 = new TalonFXHelper(CANDevices.DRIVETRAIN_RIGHT_MOTOR_2);

  pidgey = new PigeonHelper(CANDevices.PIGEON_IMU);

  leftMotor1.configFactoryDefault();
  leftMotor2.configFactoryDefault();
  rightMotor1.configFactoryDefault();
  rightMotor2.configFactoryDefault();

  leftMotor1.configFollowerStatusFrameRates();
  leftMotor2.configFollowerStatusFrameRates();
  rightMotor1.configClosedLoopStatusFrameRates();
  rightMotor2.configFollowerStatusFrameRates();

  // Create a current limit
  talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
  CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

  // Add the current limit to the motor configuration object
  leftConfig.supplyCurrLimit = talonCurrentLimit;
  rightConfig.supplyCurrLimit = talonCurrentLimit;
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
