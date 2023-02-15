// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import org.team2168.Constants;
import org.team2168.Constants.Climber;
import org.team2168.utils.TalonFXHelper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Elevator extends SubsystemBase {

  private TalonFXHelper elevatorMotorLeft;
  private TalonFXHelper elevatorMotorRight;
  private static double kArbitraryFeedForward;
  private double position;

  static Elevator instance = null;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotorLeft = new TalonFXHelper(Climber.ELEVATOR_MOTOR_LEFT); //these are placeholders
    elevatorMotorRight = new TalonFXHelper(Climber.ELEVATOR_MOTOR_RIGHT); //these are placeholders


    elevatorMotorLeft.configFactoryDefault();
    elevatorMotorRight.configFactoryDefault();
  }

  public static Elevator getInstance(){
    if (instance == null){
      instance = new Elevator();
    }
    return instance;
  }

  @Config(name = "Speed (Velocity)")
  public void setSpeedVelocity(double speed) {
    elevatorMotorLeft.set(ControlMode.Velocity, speed); //speed is a placeholder for an actual speed rate parameter
    elevatorMotorRight.set(ControlMode.Velocity, speed);
  }

  @Config(name = "Position")
  public void setPosition(double position){
    this.position = position;

    elevatorMotorLeft.set(ControlMode.MotionMagic, this.position, DemandType.ArbitraryFeedForward, kArbitraryFeedForward);
    elevatorMotorRight.set(ControlMode.MotionMagic, this.position, DemandType.ArbitraryFeedForward, kArbitraryFeedForward);
  }

  @Config(name = "Percent Output")
  public void setPercentOutput(double speed) {
    elevatorMotorLeft.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, 0.0);
  }

  public double getPosition(){
    return elevatorMotorLeft.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
