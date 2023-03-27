// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.elevator;

import java.util.function.DoubleSupplier;

import org.team2168.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveElevator extends CommandBase {
  /** Creates a new DriveElevator. */

  private Elevator elevator;
  private DoubleSupplier elevatorOutput;

  public DriveElevator(Elevator elevator, DoubleSupplier d) {
    this.elevator = elevator;
    elevatorOutput = d;

    addRequirements(elevator);
  }

  // public DriveElevator(Elevator elevator2, double testJoystickX) {
  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //elevator.setPercentOutput(elevatorPercentOutput);
  if (elevator.isInRange()) {
    elevator.setPercentOutput(elevatorOutput.getAsDouble());
  }
  else {
    elevator.setPercentOutput(0.0);
    System.out.println("not in range");
    System.out.println("position: " + elevator.getPositionIn());
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
