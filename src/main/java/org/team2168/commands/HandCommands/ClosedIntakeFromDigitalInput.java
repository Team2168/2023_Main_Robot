// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.HandCommands;

import org.team2168.subsystems.HandPneumatic;
import org.team2168.subsystems.HandWheels;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClosedIntakeFromDigitalInput extends CommandBase {
private HandWheels hand;
private HandPneumatic handPneumatic;
  public ClosedIntakeFromDigitalInput(HandWheels hand, HandPneumatic handPneumatic) {
    this.hand = hand;
    this.handPneumatic = handPneumatic;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  new OpenAndRunIntake(hand, handPneumatic);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new ClampAndStopIntake(hand, handPneumatic);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hand.isGamePieceInHand();
  }
}
