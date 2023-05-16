// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.ScoringPositions;

import org.team2168.commands.Arm.RotateArm;
import org.team2168.commands.Turret.ZeroTurret;
import org.team2168.commands.elevator.DriveElevatorToPosition;
import org.team2168.subsystems.Arm;
import org.team2168.subsystems.Elevator;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidNode extends SequentialCommandGroup {
  /** Creates a new MidConeNode. */
  Elevator elevator;
  Arm arm;
  Turret turret;
  public MidNode(Elevator elevator, Arm arm, Turret turret, Limelight limelight) {
    this.elevator = elevator;
    this.arm = arm;
    this.turret = turret;
    addCommands(new DriveElevatorToPosition(elevator, 0.5).withTimeout(1.5),
    Commands.parallel(new RotateArm(arm, 95.0),
    Commands.sequence(
      new WaitCommand(0.5),
      new DriveElevatorToPosition(elevator, -1.5).withTimeout(1.5),
      new ZeroTurret(turret, limelight))));
  }
}
