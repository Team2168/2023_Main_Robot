// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Turret;

import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Turret;
import org.team2168.subsystems.Arm;
import org.team2168.utils.Util;

import edu.wpi.first.wpilibj2.command.CommandBase;
import io.github.oblarg.oblog.annotations.Log;

public class DriveTurretWithLimelight extends CommandBase {

  private Turret turret;
  private Limelight limelight;
  private Arm arm;
  
  private double errorToleranceAngle = 0.1;
  private double limeXPos;
  private double avg_limeXPos;

  private double currentPos;
  private double targetPos;

  private double forwardSoftLimit;
  private double reverseSoftLimit;

  private double lowerArmAllowed = 60;
  private double upperArmAllowed = 140;


  private static final double LIME_KP = 0.65;

  private double armAngle;

  @Log(name = "Turn Speed")
  private double driveLimeTurn;
  /** Creates a new DriveTurretWithLimelight. */
  public DriveTurretWithLimelight(Turret turret, Limelight limelight, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.limelight = limelight;

    addRequirements(turret);
  }

  public DriveTurretWithLimelight(Turret turret, Limelight limelight, double acceptableAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.limelight = limelight;
    acceptableAngle = errorToleranceAngle;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.enableBaseCameraSettings();
    limelight.setPipeline(0);
    currentPos = turret.getEncoderPosition();
    forwardSoftLimit = Turret.getForwardSoftLimit();
    reverseSoftLimit = Turret.getReverseSoftLimit();
    avg_limeXPos = 0.0;
    limelight.enableVision(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limeXPos = limelight.getOffsetX();
    avg_limeXPos = Util.runningAverage(limeXPos, avg_limeXPos, 0.15);
    currentPos = Turret.ticksToDegrees(turret.getEncoderPosition());
    targetPos = currentPos + (avg_limeXPos * LIME_KP);
    
    armAngle = arm.getPositionDegrees();

    // arm angle is in the if statement to make sure if the arm is in positions where it would break the robot if it would turn, it won't turn    
    if (targetPos > reverseSoftLimit && targetPos < forwardSoftLimit && lowerArmAllowed <= armAngle && armAngle <= upperArmAllowed) {
      driveLimeTurn = targetPos;
    }
    else if (targetPos < reverseSoftLimit) {
      driveLimeTurn = reverseSoftLimit;
    }
    else if (targetPos > forwardSoftLimit) {
      driveLimeTurn = forwardSoftLimit;
    }
  

    turret.setRotationDegrees(driveLimeTurn);
    // System.out.println("angle: " + driveLimeTurn);
    // System.out.println("limelight target ang: " + limeXPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setVelocity(0.0);
    limelight.pauseLimelight();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
