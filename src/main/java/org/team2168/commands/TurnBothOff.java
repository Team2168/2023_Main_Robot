// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team2168.subsystems.LEDs;


public class TurnBothOff extends CommandBase {
  /** Creates a new TurnYellowOn. */

  private LEDs leds;
  private boolean redIsOn;
  private boolean blueIsOn;
  private boolean greenIsOn;

  public TurnBothOff(LEDs leds, boolean redIsOn, boolean blueIsOn, boolean greenIsOn) {
    this.leds = leds;
    this.redIsOn = redIsOn;
    this.blueIsOn = blueIsOn;
    this.greenIsOn = greenIsOn;

    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leds.setLED(false, false, false);
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
