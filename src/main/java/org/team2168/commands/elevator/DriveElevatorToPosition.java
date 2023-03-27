// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.elevator;

//import java.util.function.DoubleSupplier;

import org.team2168.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveElevatorToPosition extends CommandBase {
  /** Creates a new DriveElevatorToPosition. */

  private Elevator elevator;
  private static double inches;
  private static double errorTolerance = 0.5;

  public DriveElevatorToPosition(Elevator elevator, double in) {
    this.elevator = elevator;
    inches = in;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //elevator.setSpeedVelocity(speedInInches);
    elevator.setPosition(inches);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setPercentOutput(0.0);
    //elevator.setSpeedVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (elevator.getPositionIn() >= (inches - errorTolerance) && elevator.getPositionIn() <= (inches + errorTolerance));
  }
}
