// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.led;

import org.team2168.subsystems.LEDs;
import org.team2168.subsystems.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LEDRainbow extends CommandBase {
  /** Creates a new LEDRainbow. */

  LEDs LED;
  Limelight limelight;
  double timecount = 0;
  
  public LEDRainbow(LEDs LED, Limelight limelight, double timecount) {
    this.LED = LED;
    this.limelight = limelight;
    this.timecount = timecount;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(LED, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limelight.getCurrentPipeline() == 3) {
      LED.setLED(true, true, false);
    } else if (limelight.getCurrentPipeline() == 4) {
      LED.setLED(true, false, true);
    } else if (limelight.getCurrentPipeline() != 3 && limelight.getCurrentPipeline() != 4 && timecount >= 0 && timecount <= 5) { 
      ++timecount;
        LED.setLED(true, false, false);
      } else if (timecount >= 5 && timecount <= 10) {
        LED.setLED(true, false, true);
        } else if (timecount >= 10 && timecount <= 15) {
          LED.setLED(false, false, true);
          } else if (timecount >= 15 && timecount <= 20) {
            LED.setLED(false, true, true);
            } else if (timecount >= 20 && timecount <= 25) {
              LED.setLED(false, true, false);
              } else if (timecount >= 25 && timecount <= 30) {
                LED.setLED(true, true, false);
                } else if (timecount > 30) {
                  timecount = 0;
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
