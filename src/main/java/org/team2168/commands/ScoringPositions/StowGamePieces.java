// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.ScoringPositions;

import org.team2168.commands.Arm.RotateArm;
import org.team2168.commands.elevator.DriveElevatorToPosition;
import org.team2168.subsystems.Arm;
import org.team2168.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StowGamePieces extends SequentialCommandGroup {
  /** Creates a new StowGamePieces. */
  Elevator elevator;
  Arm arm;

  public StowGamePieces(Elevator elevator, Arm arm) {
    this.elevator = elevator;
    this.arm = arm;

    addCommands(new DriveElevatorToPosition(elevator, 0.5).withTimeout(0.5),
    new RotateArm(arm, 90.0).withTimeout(0.5),
    Commands.parallel(new DriveElevatorToPosition(elevator, -22.5),
    new RotateArm(arm, 155.0)));
  }
}
