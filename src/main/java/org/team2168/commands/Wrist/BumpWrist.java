// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Wrist;

import org.team2168.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BumpWrist extends CommandBase {
  /** Creates a new BumpWrist. */

  private Wrist wrist;
  private double degrees;
  
  /**
   * Bumps the wrist's position
   * @param wrist the Wrist subsystem
   * @param degrees the amount to bump the wrist (degrees)
   */
  public BumpWrist(Wrist wrist, double degrees) {
    this.wrist = wrist;
    this.degrees = degrees;

    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var setpoint = wrist.getSetpoint() + degrees;
    wrist.setRotationDegrees(setpoint);
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
