// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HandWheelsTwo extends SubsystemBase {
private CANSparkMax intakeLeftMotor;
private CANSparkMax intakeRightMotor;
private SparkMaxPIDController leftController = intakeLeftMotor.getPIDController();
private int leftCurrentLimit = 10;
private int rightCurrent

  public HandWheelsTwo() {
    //775 motors which are brushed
    intakeLeftMotor = new CANSparkMax(Constants.CANDevices.INTAKE_LEFT_MOTOR, MotorType.kBrushed);
    intakeRightMotor = new CANSparkMax(Constants.CANDevices.INTAKE_RIGHT_MOTOR, MotorType.kBrushed);

    intakeLeftMotor.restoreFactoryDefaults();
    intakeRightMotor.restoreFactoryDefaults();

    intakeLeftMotor.setSmartCurrentLimit(10);
    intakeRightMotor.setSmartCurrentLimit(10);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
