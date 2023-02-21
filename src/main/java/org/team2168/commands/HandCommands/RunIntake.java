// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.HandCommands;

import org.team2168.Constants;
import org.team2168.subsystems.HandWheels;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunIntake extends CommandBase {
  private HandWheels hand;
  private double speedValue;

  public RunIntake(HandWheels hand, double speedValue) {
    this.hand = hand;
    this.speedValue = speedValue;
    addRequirements(hand);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hand.set(speedValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hand.set(Constants.MotorSpeeds.STOP_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
