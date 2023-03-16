// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.drivetrain;

import java.util.function.DoubleSupplier;

import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TankDrive extends CommandBase {
  /** Creates a new TankDrive. */
  Drivetrain drivetrain;
  DoubleSupplier leftSpeed;
  DoubleSupplier rightSpeed;
  public TankDrive(Drivetrain drivetrain, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
    this.drivetrain = drivetrain;
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.tankDrive(leftSpeed.getAsDouble(), rightSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.tankDrive(0.0,0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
