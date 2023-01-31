// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;

import org.team2168.Constants;
import org.team2168.utils.Gains;
import org.team2168.utils.TalonFXHelper;



public class Turret extends SubsystemBase {

  private static TalonFXHelper turretMotor;
  private static Turret instance = null;

  private static final double TICKS_PER_REV = 2048;
  private static final double GEAR_RATIO = 280.0/18.0 * 36.0/12.0; // 2022 turret gear ratio TODO: fix later
  private static final double TICKS_PER_TURRET_ROTATION = TICKS_PER_REV * GEAR_RATIO;
  private static double setpoint = 0.0;

  private static final double TICKS_PER_SECOND = TICKS_PER_TURRET_ROTATION;
  private static final double TICKS_PER_100_MS = TICKS_PER_SECOND / 10.0;
  private static final double ONE_HUNDRED_MS_PER_MINUTE = 100.0 / 60.0;

  // 2022 values, TODO: Update to new robots requirements.
  private static final int MIN_ROTATION_TICKS = -73400; 
  private static final int MAX_ROTATION_TICKS = 52200; 

  private static final double MIN_ROTATION_DEGREES = ticksToDegrees(-73400.0);
  private static final double MAX_ROTATION_DEGREES = ticksToDegrees(52200.0);
  private static final double TOTAL_ROTATION_DEGREES = Math.abs(MIN_ROTATION_DEGREES) + Math.abs(MAX_ROTATION_DEGREES);

    /** Creates a new Turret. */
  public Turret() {
    turretMotor = new TalonFXHelper(Constants.CANDevices.TURRET_MOTOR);
  
 }

 public static Turret getInstance() {
  if (instance == null)
    instance = new Turret();
    return instance;
 }

  private static double ticksToDegrees(double ticks) {
    return (ticks / TICKS_PER_TURRET_ROTATION) * 360.0;
  }

  private static double degreesToEncoderTicks(double degrees) {
    return (degrees/360.0) * TICKS_PER_TURRET_ROTATION;
  }

  private static double ticksPer100msToDegreesPerSecond(double ticks) {
    return ticksToDegrees(ticks) * 10.0;
  }

  public static double degreesPerSecondToTicksPer100ms(double degrees) {
    return degreesToEncoderTicks(degrees) / 10.0;
  }

  public void setRotationDegrees(double degrees) {
    var demand = MathUtil.clamp(degrees, ticksToDegrees(MIN_ROTATION_TICKS), ticksToDegrees(MAX_ROTATION_TICKS));
    setpoint = degrees;
    turretMotor.set(ControlMode.MotionMagic, degreesToEncoderTicks(demand));
  }

  public void setVelocity(double degrees) {
    turretMotor.set(ControlMode.Velocity, degreesPerSecondToTicksPer100ms(degrees));
  }

  public void setSpeed(double speed) {
    turretMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
