// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import java.util.function.DoubleSupplier;

import org.team2168.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveElevator extends CommandBase {
  /** Creates a new DriveElevator. */

  private Elevator elevator;
  private double elevatorSpeed;

  public DriveElevator(Elevator elevator, double d) {
    this.elevator = elevator;
    elevatorSpeed = d;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setPercentOutput(elevatorSpeed);;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setSpeedVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (elevator.getPosition() == 0);
  }
}
