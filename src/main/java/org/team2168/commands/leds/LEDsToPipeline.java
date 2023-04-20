// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.leds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team2168.subsystems.LEDs;
import org.team2168.subsystems.Limelight;

public class LEDsToPipeline extends CommandBase {
  /** Creates a new LEDsToPipeline. */
  private LEDs leds;
  private Limelight limelight;

  public LEDsToPipeline(LEDs leds, Limelight limelight) {
    this.leds = leds;
    this.limelight = limelight;

    addRequirements(leds, limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.enableBaseCameraSettings();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limelight.getCurrentPipeline() == 3){
      leds.setLED(true, true, false);
    }
    else if (limelight.getCurrentPipeline() == 4){
      leds.setLED(true, false, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
