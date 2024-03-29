// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.elevator;

import org.team2168.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendLock extends CommandBase {

  Elevator carriageLock;

  /** Creates a new ExtendLock. */
  public ExtendLock(Elevator carriageLock) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.carriageLock = carriageLock;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    carriageLock.extendLock();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return carriageLock.isLockExtended();
  }
}
