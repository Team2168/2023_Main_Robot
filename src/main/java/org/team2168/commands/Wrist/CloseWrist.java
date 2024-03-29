// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Wrist;

import org.team2168.subsystems.WNE_Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CloseWrist extends CommandBase {
  /** Creates a new RetractWrist. */
  private WNE_Wrist wrist;

  public CloseWrist(WNE_Wrist wrist) {
    this.wrist = wrist;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.extend();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wrist.isExtended();
  }
}
