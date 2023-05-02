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
    ++timecount;
    if (limelight.getCurrentPipeline() == Limelight.Pipeline.SCAN_FOR_CUBE.pipelineValue) {
       // System.out.println("Checking for Pipeline 3");
       LED.setLED(true, true, false);
    } else if (limelight.getCurrentPipeline() == Limelight.Pipeline.SCAN_FOR_CONE.pipelineValue) {
       // System.out.println("Checking for Pipeline 4");
       LED.setLED(true, false, true);
    }  else if (limelight.getCurrentPipeline() != 3 && limelight.getCurrentPipeline() != 4) { 
      LED.rainbowLED(); }
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
