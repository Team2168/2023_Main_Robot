// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.HandCommands;

import org.team2168.Constants;
import org.team2168.subsystems.HandPneumatic;
import org.team2168.subsystems.HandWheels;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunIntakeUntilGamePieceIn extends CommandBase {
private HandWheels hand;
private HandPneumatic handPneumatic;
  public RunIntakeUntilGamePieceIn(HandWheels hand, HandPneumatic handPneumatic) {
    this.hand = hand;
    this.handPneumatic = handPneumatic;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hand.set(Constants.MotorSpeeds.FORWARD_INTAKE_VELOCITY);
    handPneumatic.setOpen();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hand.set(Constants.MotorSpeeds.STOP_SPEED);
    handPneumatic.setClamp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // handPneumatic.isIntakeClamped();
    return hand.isGamePieceInHand();
  }
}
