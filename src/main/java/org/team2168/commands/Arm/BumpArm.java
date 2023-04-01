// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Arm;

import org.team2168.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BumpArm extends CommandBase {
  /** Creates a new BumpArm. */
  private Arm arm;
  private double degrees;

  /**
   * Bumps the arm's position
   * @param arm the Arm subsystem
   * @param degrees the amount to bump the arm (degrees)
   */
  public BumpArm(Arm arm, double degrees) {
    this.arm = arm;
    this.degrees = degrees;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var setpoint = arm.getSetpoint() + degrees;
    arm.setRotationDegrees(setpoint);
    
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
