// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.drivetrain;

import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AdjustOnChargeStation extends CommandBase {
  /** Creates a new AdjustOnChargeStation. */
  Drivetrain drivetrain;
  double pitchErrorTolerance = 0.5;
  double maxChargingStationDeg = 15.0;
  double lowestSpeed = 0.08;
  double highestSpeed = 0.25;
  int balancedLoops;
  PIDController pid;

  //gains
  private static final double P = 0.01;
  private static final double I = 0.0;
  private static final double D = 0.0;
  private static final double MAX_INTEGRAL = 1.0;

  public AdjustOnChargeStation(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setMotorsBrakeAutos();
    pid = new PIDController(P, I, D);
    pid.setTolerance(pitchErrorTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (drivetrain.getPitch() > pitchErrorTolerance) {
      // drivetrain.arcadeDrive(MathUtil.clamp(0.25*drivetrain.getPitch()/maxChargingStationDeg, -highestSpeed, -lowestSpeed), 0.0);
      drivetrain.arcadeDrive(pid.calculate(drivetrain.getPitch()), 0.0);
      balancedLoops = 0;
    }
    else if (drivetrain.getPitch() < -pitchErrorTolerance) {
      // drivetrain.arcadeDrive(MathUtil.clamp(0.25*drivetrain.getPitch()/maxChargingStationDeg, lowestSpeed, highestSpeed), 0.0);
      drivetrain.arcadeDrive(pid.calculate(drivetrain.getPitch()), 0.0);
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
    return (balancedLoops > 20);
  }
}
