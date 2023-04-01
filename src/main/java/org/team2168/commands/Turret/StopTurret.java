// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Turret;

import java.util.function.DoubleSupplier;

import org.team2168.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class StopTurret extends CommandBase {
  /** Creates a new StopTurret. */

  Turret turret;
  DoubleSupplier speed;
  
  public StopTurret(Turret t, DoubleSupplier s) {
    t = turret;
    s = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(t);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setSpeed(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
