// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team2168.subsystems.LEDs;

public class TurnAllOnOrOff extends CommandBase {
  /** Creates a new TurnAllOnOrOff. */

  LEDs leds;
  boolean greenIsOn;
  boolean redIsOn;
  boolean blueIsOn;

  public TurnAllOnOrOff(LEDs leds, boolean greenIsOn, boolean redIsOn, boolean blueIsOn) {
    leds = this.leds;
    greenIsOn = this.greenIsOn;
    redIsOn = this.redIsOn;
    blueIsOn = this.blueIsOn;

    addRequirements(leds);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leds.greenOnOff(greenIsOn);
    leds.redOnOff(redIsOn);
    leds.blueOnOff(blueIsOn); 
    //if some of these were to be combined together it would create purple and yellow (this would make all the other commands unneeded)
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
