// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import org.team2168.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveElevatorToPosition extends CommandBase {
  /** Creates a new DriveElevatorToPosition. */

  private static Elevator elevator;
  private static double inches;

  public DriveElevatorToPosition(Elevator elevator, double inch) {
    this.elevator = elevator;
    inches = inch;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setPosition(inches);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setPercentOutput(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
