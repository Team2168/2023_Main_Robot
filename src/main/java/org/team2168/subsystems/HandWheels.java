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
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class HandWheels extends SubsystemBase implements Loggable {
  private static HandWheels instance = null;
  private CANSparkMax intakeLeftMotor;
  private CANSparkMax intakeRightMotor;
  private DigitalInput handLineBreak;
  private boolean leftInvert = false;
  private boolean rightInvert = true;

  private final int CURRENT_LIMIT = 20;
  private final int VOLTAGE_COMPENSATION = 10;

  public HandWheels() {
    // 775 motors which are brushed
    intakeLeftMotor = new CANSparkMax(Constants.CANDevices.INTAKE_LEFT_MOTOR, MotorType.kBrushed);
    intakeRightMotor = new CANSparkMax(Constants.CANDevices.INTAKE_RIGHT_MOTOR, MotorType.kBrushed);

    handLineBreak = new DigitalInput(Constants.DIO.HAND_CHANNEL);

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

    intakeLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 255); // 255 is the samef periodMS value used in
                                                                         // TalonFXHelperClass.
    //to know what kStatus(0,1,2) do, go to definiton of PeriodicFrame.
    intakeLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 255);
    intakeLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 255);

    intakeRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 255);
    intakeRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 255);
    intakeRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 255);
  }

  public static HandWheels getInstance() {
    if (instance == null) {
      instance = new HandWheels();
    }
    return instance;
  }

  public void set(double volts) {
    intakeLeftMotor.setVoltage(volts);

  }

  @Log(name = "output", rowIndex = 2, columnIndex = 1)
  public double getOutput() {
    return intakeLeftMotor.getAppliedOutput(); // speed (-1.0 - 1.0) according to javadoc comment
  }

  @Log(name = "voltage", rowIndex = 2, columnIndex = 1)
  public double getVoltage() {
    return intakeLeftMotor.getBusVoltage(); // speed (-1.0 - 1.0) according to javadoc comment
  }

  @Log(name = "IsGamePieceInHand: ", rowIndex = 1, columnIndex = 1)
  public boolean isGamePieceInHand() {
    return !handLineBreak.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
