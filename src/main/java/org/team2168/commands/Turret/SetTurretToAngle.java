// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Turret;

import org.team2168.subsystems.Turret;
import org.team2168.utils.Util;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetTurretToAngle extends CommandBase {

  Turret turret;
  Double targetPositionDegrees;
  Double acceptableErrorDegrees = 0.1;

  private double error;

  /** Creates a new SetTurretToAngle. */
  public SetTurretToAngle(Turret t, Double targetPosition) {

    turret = t;
    targetPositionDegrees = targetPosition;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(t);
  }

  public SetTurretToAngle(Turret t, Double a, Double acceptableError) {
    acceptableErrorDegrees = acceptableError;

    addRequirements(t);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    error = turret.getEncoderPosition() - targetPositionDegrees;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setRotationDegrees(targetPositionDegrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentError = turret.getEncoderPosition() - targetPositionDegrees;
    error = Util.runningAverage(currentError, error, 0.85);

    return Math.abs(error) < acceptableErrorDegrees;

  }
}
