// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;
import org.team2168.Constants.Climber;
import org.team2168.utils.TalonFXHelper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Elevator extends SubsystemBase {

  private TalonFXHelper elevatorMotorLeft;
  private TalonFXHelper elevatorMotorRight;

  static Elevator instance = null;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotorLeft = new TalonFXHelper(Climber.ELEVATOR_MOTOR_LEFT); //these are placeholders
    elevatorMotorRight = new TalonFXHelper(Climber.ELEVATOR_MOTOR_RIGHT); //these are placeholders
  }

  public static Elevator getInstance(){
    if (instance == null){
      instance = new Elevator();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
