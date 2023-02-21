// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class HandWheels extends SubsystemBase {
  private static HandWheels instance;
  private CANSparkMax intakeLeftMotor;
  private CANSparkMax intakeRightMotor;
  private DigitalInput input;
  private boolean leftInvert = false;
  private boolean rightInvert = true;

  private SparkMaxPIDController leftController;
  private final double KP = 1.0;
  private final double KI = 0.0;
  private final double KD = 0.0;
  private final double KF = 0.0025;
  private final double I_ZONE = 0.0;
  private final int CURRENT_LIMIT = 20;
  private final int VOLTAGE_COMPENSATION = 10;

  public HandWheels() {
    // 775 motors which are brushed
    intakeLeftMotor = new CANSparkMax(Constants.CANDevices.INTAKE_LEFT_MOTOR, MotorType.kBrushed);
    intakeRightMotor = new CANSparkMax(Constants.CANDevices.INTAKE_RIGHT_MOTOR, MotorType.kBrushed);
    leftController = intakeLeftMotor.getPIDController();
    input = new DigitalInput(Constants.DIO.HAND_CHANNEL);

    intakeLeftMotor.restoreFactoryDefaults();
    intakeRightMotor.restoreFactoryDefaults();

    intakeLeftMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    intakeRightMotor.setSmartCurrentLimit(CURRENT_LIMIT);

    intakeLeftMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);
    intakeRightMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);

    intakeLeftMotor.setIdleMode(IdleMode.kBrake);
    intakeRightMotor.setIdleMode(IdleMode.kBrake);

    intakeLeftMotor.setInverted(leftInvert);
    intakeRightMotor.follow(intakeLeftMotor, rightInvert);

    leftController.setP(KP);
    leftController.setI(KI);
    leftController.setD(KD);
    leftController.setFF(KF);
    leftController.setIZone(I_ZONE);

  }

  public static HandWheels getInstance() {
    if (instance == null) {
      instance = new HandWheels();
    }
    return instance;
  }

  public void set(double speed) {
    intakeLeftMotor.set(speed);
    leftController.setReference(speed, ControlType.kDutyCycle);
  }

  @Log(name = "Intake Velocity: ", tabName = "IntakeTab", methodName = "getVelocity()", width = 2, height = 2, rowIndex = 1, columnIndex = 1)
  public double getSpeed() {
    return intakeLeftMotor.get(); // speed (-1.0 - 1.0) according to javadoc comment
  }

  @Config(name = "IsGamePieceInHand: ")
  public boolean isGamePieceInHand() {
    return !input.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
