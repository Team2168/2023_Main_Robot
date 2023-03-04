// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.drivetrain;

import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AdjustOnChargeStation extends CommandBase {
  /** Creates a new AdjustOnChargeStation. */
  Drivetrain drivetrain;
  double pitchErrorTolerance = 0.5;
  double maxChargingStationDeg = 15.0;
  double lowestSpeed;
  double highestSpeed;
  int balancedLoops;
  public AdjustOnChargeStation(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (drivetrain.getPitch() < -pitchErrorTolerance) {
      drivetrain.arcadeDrive(MathUtil.clamp(0.25*drivetrain.getPitch()/maxChargingStationDeg, -0.25, -0.08), 0.0);
      balancedLoops = 0;
    }
    else if (drivetrain.getPitch() > pitchErrorTolerance) {
      drivetrain.arcadeDrive(MathUtil.clamp(0.25*drivetrain.getPitch()/maxChargingStationDeg, 0.08, 0.25), 0.0);
      balancedLoops = 0;
    }
    else {
      drivetrain.arcadeDrive(0.0, 0.0);
      ++balancedLoops;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (balancedLoops > 10);
  }
}
