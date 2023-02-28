// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import java.security.KeyFactory;

import org.team2168.Constants;
import org.team2168.Constants.Climber;
import org.team2168.utils.TalonFXHelper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Elevator extends SubsystemBase {

  private static final double kI = 0.3; //intergral (TODO: replace placeholder)
  private static final double kD = 0.3; //derivative (TODO: replace placeholder)
  private static final double kF = 0.3; //feedforward: constant output added on which counteracts forces (TODO: replace placeholder)
  private static final double kP = 0.5; //proportional: a proportion of the input (TODO: replace placeholder)
  private static final double kArbitraryFeedForward = 0.1;

  private static final int kTimeoutMs = 30; //how long it takes for the config to configure in Ms
  private static final int kPIDLoopIdx = 0; //constant for id purposes

  private static final double CURRENT_LIMIT = 2; //(TODO: replace placeholder), it limits when the feature is activited (in amps)
  private static final double THRESHOLD_CURRENT = 3; //(TODO: replace placeholder), it tells what the threshold should be for the limit to be activited (in amps)
  private static final double THRESHOLD_TIME = 3; //(TODO: replace placeholder), time in seconds of when the limiting should happen after the threshold has been overreached

  private static final double TIME_UNITS_OF_VELOCITY = 0.1; //in seconds 
  private static final double TICKS_PER_REV = 2048;
  private static final double GEAR_RATIO = (1/3.75); 
  private static final double SPROCKET_RADIUS = 0.25; 
  private static final double INCHES_PER_REV = SPROCKET_RADIUS * 2 * Math.PI;


  private static final double kPeakOutput = 1.0;
  private static final double NEUTRAL_DEADBAND = 0.001; 
  private static final double ACCELERATION_LIMIT = 5 * TIME_UNITS_OF_VELOCITY; //(TODO: this could be a pontinteal placeholder)

  private static TalonFXInvertType kInvertType = TalonFXInvertType.Clockwise; //this inverts the rotation of the motors so that the shaft goes up (clockwise)

  private TalonFXHelper elevatorMotorLeft;
  private TalonFXHelper elevatorMotorRight;
  //private double position; //height in inches
  private SupplyCurrentLimitConfiguration talonCurrentLimit;

  // private boolean kSensorPhase;
  // private boolean kMotorInvert;

  static Elevator instance = null;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotorLeft = new TalonFXHelper(Climber.ELEVATOR_MOTOR_LEFT); //these are placeholder constant values
    elevatorMotorRight = new TalonFXHelper(Climber.ELEVATOR_MOTOR_RIGHT); //these are placeholder constant values

    elevatorMotorLeft.configNeutralDeadband(NEUTRAL_DEADBAND);
    elevatorMotorRight.configNeutralDeadband(NEUTRAL_DEADBAND);
    elevatorMotorLeft.setNeutralMode(NeutralMode.Brake);
    elevatorMotorRight.setNeutralMode(NeutralMode.Brake);

    //elevatorMotorLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
    //elevatorMotorLeft.setSensorPhase(kSensorPhase);
    //elevatorMotorLeft.setInverted(kMotorInvert);

    //sets the gains
    elevatorMotorLeft.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
    elevatorMotorLeft.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
    elevatorMotorLeft.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
    elevatorMotorLeft.config_kD(kPIDLoopIdx, kD, kTimeoutMs);
    elevatorMotorLeft.configMotionAcceleration(ACCELERATION_LIMIT);

    talonCurrentLimit = new SupplyCurrentLimitConfiguration(true, CURRENT_LIMIT, THRESHOLD_CURRENT, THRESHOLD_TIME);

    //puts limis on the input (configs)
    elevatorMotorRight.configSupplyCurrentLimit(talonCurrentLimit);
    elevatorMotorLeft.configSupplyCurrentLimit(talonCurrentLimit);

    //this tells the second motor to do the same things as the first motor at the same exact time
    elevatorMotorRight.set(ControlMode.Follower, Constants.Climber.ELEVATOR_MOTOR_LEFT);
    elevatorMotorRight.setInverted(InvertType.OpposeMaster);

    elevatorMotorLeft.configFactoryDefault();
    elevatorMotorRight.configFactoryDefault();
  }

  public static Elevator getInstance(){
    if (instance == null){
      instance = new Elevator();
    }
    return instance;
  }

  public static double degreesToTicks(double degrees){
    return (degrees / 360 * TICKS_PER_REV);
  }

  public static double ticksToDegrees(double ticks){
    return (ticks / TICKS_PER_REV * 360);
  }

  public static double inchesToTicks(double inches){
    return(inches/INCHES_PER_REV) * GEAR_RATIO * TICKS_PER_REV;
  }

  public static double ticksToInches(double ticks){
    return (ticks / TICKS_PER_REV) /GEAR_RATIO * INCHES_PER_REV;
  }

  @Log(name = "Speed (Velocity)")
  public void setSpeedVelocity(double speed) {
    elevatorMotorLeft.set(ControlMode.Velocity, inchesToTicks(speed) * TIME_UNITS_OF_VELOCITY); //the "speed" parameter is the rate of the movement per second (in inches)
  }

  @Log(name = "Position")
  public void setPosition(double inches){
    //this.position = position;

    elevatorMotorLeft.set(ControlMode.MotionMagic, inchesToTicks(inches), DemandType.ArbitraryFeedForward, kArbitraryFeedForward);
  }

  @Log(name = "Percent Output")
  public void setPercentOutput(double speed) {
    elevatorMotorLeft.set(ControlMode.PercentOutput, INCHES_PER_REV, DemandType.ArbitraryFeedForward, 0.0);
  }


  @Log()
  public void setToZero(){
    elevatorMotorLeft.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
  }

  public double getPosition(){
    return elevatorMotorLeft.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
