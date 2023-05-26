// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Turret;
  import org.team2168.utils.Util;
  
  
  public class ZeroTurret extends CommandBase {
  
    private Turret turret;
    private Limelight lime;
    double targetPositionDegrees;
    double acceptableErrorDegrees = 0.5;
    double angleOffset = 120.0;
  
    private double error;
  
    /** Creates a new SetTurretToAngle. */
    public ZeroTurret(Turret t, Limelight lime) {
  
      turret = t;
      this.lime = lime;
      targetPositionDegrees = 0.0;
      // Use addRequirements() here to declare subsystem dependencies.
  
      addRequirements(t);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      // error = Turret.ticksToDegrees(turret.getEncoderPosition()) - (targetPositionDegrees);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      turret.setRotationDegrees(targetPositionDegrees + angleOffset);
      lime.pauseLimelight();
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      turret.setSpeed(0.0);
      System.out.println("ending");
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      // double currentError = turret.getEncoderPosition() - targetPositionDegrees;
      // error = Util.runningAverage(currentError, error, 0.85);
  
      // return Math.abs(error) < acceptableErrorDegrees;
      // return Math.abs(Turret.ticksToDegrees(turret.getEncoderPosition()) - angleOffset) < 0.5;
      return false;
  
    }
  }
  