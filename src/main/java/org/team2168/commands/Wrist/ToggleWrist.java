// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Wrist;

import java.util.Set;

import org.team2168.commands.Turret.DriveTurretWithLimelight;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.WNE_Wrist;
import org.team2168.subsystems.Turret;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleWrist extends CommandBase {
  /** Creates a new ToggleWrist. */
  private WNE_Wrist wrist;
  private DoubleSolenoid.Value startingValue;
  private Limelight limelight;


  public ToggleWrist(WNE_Wrist wrist, Limelight limelight) {
    this.wrist = wrist;
    this.limelight = limelight;
    
    addRequirements(wrist, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.toggleWrist();
    startingValue = wrist.getWristState();

    if(startingValue == DoubleSolenoid.Value.kReverse) {
      limelight.setPipeline(Limelight.Pipeline.PIPELINE_FIVE.pipelineValue);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("command is running");

    
  }
    


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(startingValue == DoubleSolenoid.Value.kForward)
    //   return wrist.isRetracted();
    // if(startingValue == DoubleSolenoid.Value.kReverse)
    //   return wrist.isExtended();s
    // else
    //   return true;
    
    return true;
  }
}
