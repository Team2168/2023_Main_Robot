// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Wrist;

import org.team2168.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateWrist extends CommandBase {
  /** Creates a new RotateWrist. */

  private Wrist wrist;
  private double targetPosition;
  private double errorTolerance;

  /**
   * Rotates the wrist to a specified position
   * @param wrist the Wrist subsystem
   * @param targetPosition the position for the wrist to move to (degrees)
   */
  public RotateWrist(Wrist wrist, double targetPosition) {
    this(wrist, targetPosition, 1.0);

    addRequirements(wrist);
  }

  public RotateWrist(Wrist wrist, double targetPosition, double errorTolerance) {
    this.wrist = wrist;
    this.targetPosition = targetPosition;
    this.errorTolerance = errorTolerance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.setRotationDegrees(targetPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (wrist.getControllerError() < errorTolerance);
  }
}
