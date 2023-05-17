// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Turret;

import java.util.function.DoubleSupplier;

import org.team2168.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTurretWithJoystick extends CommandBase {

  private DoubleSupplier speed;
  private Turret turret;
  /** Creates a new DriveTurretWithJoystick. */
  public DriveTurretWithJoystick(Turret t, DoubleSupplier s) {
    turret = t;
    speed = s;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if ((Turret.ticksToDegrees(turret.getEncoderPosition())) < Turret.getForwardSoftLimit() || 
    (Turret.ticksToDegrees(turret.getEncoderPosition())) > Turret.getReverseSoftLimit()){
      turret.setSpeed(speed.getAsDouble());
    }
    else if ((Turret.ticksToDegrees(turret.getEncoderPosition()) > Turret.getForwardSoftLimit()) &&
    (speed.getAsDouble() < 0.0)) {
      turret.setSpeed(speed.getAsDouble()); // assures if our turret gets knocked out of its soft limits it can get back where to its supposed to be
    }
    else if ((Turret.ticksToDegrees(turret.getEncoderPosition()) < Turret.getReverseSoftLimit()) &&
    (speed.getAsDouble() > 0.0)) {
      turret.setSpeed(speed.getAsDouble());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
