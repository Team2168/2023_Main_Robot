// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.drivetrain;

import java.util.function.DoubleSupplier;

import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */
  Drivetrain drivetrain;
  DoubleSupplier rotationSpeed;
  DoubleSupplier forwardSpeed;
  public ArcadeDrive(Drivetrain drivetrain, DoubleSupplier forwardSpeed, DoubleSupplier rotationSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.forwardSpeed = forwardSpeed;
    this.rotationSpeed = rotationSpeed;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDrive(forwardSpeed.getAsDouble(), rotationSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
