// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team2168.subsystems.LEDs;;

public class TurnGreenOn extends CommandBase {

  private LEDs leds;
  private boolean isOn;

  /** Creates a new TurnGreenOn. */
  public TurnGreenOn(LEDs leds, boolean isOn) {
    leds = this.leds;
    isOn = this.isOn;

    addRequirements(leds);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leds.greenOnOff(isOn);
    leds.redOnOff(false);
    leds.blueOnOff(false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
