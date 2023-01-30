// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;
import org.team2168.utils.TalonFXHelper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HandWheels extends SubsystemBase {

  private TalonFXHelper intakeLeftMotor;
  private TalonFXHelper intakeRightMotor;
  private DigitalInput input;
  private static HandWheels instance = null;
  private SupplyCurrentLimitConfiguration limitConfig;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CURRENT_LIMIT = 25;
  private final double THRESHOLD_LIMIT = 30;
  private final double THRESHOLD_TIME = 0.2;
  private final double KP = 1.0;
  private final double KI = 0.0;
  private final double KD = 0.0;
  private final double KF = 0.0025;
  private final double NEUTRAL_DEADBAND = 0.001;
  private TalonFXInvertType leftMotorInvert = TalonFXInvertType.Clockwise;
  private TalonFXInvertType rightMotorInvert = TalonFXInvertType.OpposeMaster; //CounterClockwise if change is needed
  private final int PID_SLOT_X = 0;
  private final double GEAR_RATIO = 1; //placeholder
  private final double ARBITRARY_FEED_FORWARD = 0.0025; //prevent motors from stalling if collecting game pieces

  public HandWheels() {

    intakeLeftMotor = new TalonFXHelper(Constants.CANDevices.INTAKE_LEFT_MOTOR);
    intakeRightMotor = new TalonFXHelper(Constants.CANDevices.INTAKE_RIGHT_MOTOR);
    input = new DigitalInput(Constants.DIO.HAND_CHANNEL);

    intakeRightMotor.configFactoryDefault();
    intakeLeftMotor.configFactoryDefault();

    limitConfig = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT,
        THRESHOLD_LIMIT, THRESHOLD_TIME);

    intakeRightMotor.configSupplyCurrentLimit(limitConfig);
    intakeLeftMotor.configSupplyCurrentLimit(limitConfig);

    intakeRightMotor.setInverted(rightMotorInvert);
    intakeLeftMotor.setInverted(leftMotorInvert);

    intakeRightMotor.setNeutralMode(NeutralMode.Brake);
    intakeLeftMotor.setNeutralMode(NeutralMode.Brake);

    intakeRightMotor.follow(intakeLeftMotor, FollowerType.PercentOutput);

    intakeLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    intakeLeftMotor.config_kP(PID_SLOT_X, KP);
    intakeLeftMotor.config_kI(PID_SLOT_X, KI);
    intakeLeftMotor.config_kD(PID_SLOT_X, KD);
    intakeLeftMotor.config_kF(PID_SLOT_X, KF);
    intakeLeftMotor.configNeutralDeadband(NEUTRAL_DEADBAND);

    intakeRightMotor.configClosedLoopStatusFrameRates();
    intakeLeftMotor.configClosedLoopStatusFrameRates();

  }

  public static HandWheels getInstance(){
    if(instance == null){
      instance = new HandWheels();
    }
    return instance;
  }

  public void setPercentOutput(double percentOutput){
    intakeLeftMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setVelocity(double rpm) {
    intakeLeftMotor.set(ControlMode.Velocity, rpmToTicksPerHundredMS(rpm));
  }

  public double rpmToTicksPerHundredMS(double rpm) {
    return (rpm / 600) * (2048 * GEAR_RATIO);
  }

  public double ticksPerHundredMsToRPM(double ticks) {
    return (ticks * 600) / (2048 / GEAR_RATIO);
  }
 
  public double getVelocity(){
    return ticksPerHundredMsToRPM(intakeLeftMotor.getSelectedSensorVelocity());
  }


  public boolean isGamePieceInHand(){
    return !input.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
