// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import org.team2168.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveElevatorToZero extends CommandBase {
  /** Creates a new DriveElevatorToPosition. */

  private Elevator elevator;

  public DriveElevatorToZero(Elevator elevator) {
    this.elevator = elevator;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setToZero();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted){
      elevator.setPercentOutput(0.0);
      elevator.setToZero();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (elevator.getPositionIn() == 0);
  }
}
