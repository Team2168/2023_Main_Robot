// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Arm;

import org.team2168.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateArm extends CommandBase {
  /** Creates a new RotateArm. */
  
  private Arm arm;
  private double degrees;
  private double errorTolerance = 1.0;
  private double angleOffset = 28.0;

  /**
   * Rotates the arm to a position
   * 
   * @param arm the Arm subsystem
   * @param degrees the position for the arm to move to (degrees)
   */
  public RotateArm(Arm arm, double degrees) {
    this.arm = arm;
    this.degrees = degrees;

    addRequirements(arm);
  }

  public RotateArm(Arm arm, double degrees, double errorTolerance) {
    this.arm = arm;
    this.degrees = degrees;
    this.errorTolerance = errorTolerance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println(degrees);
    // System.out.println((arm.getControllerError() < errorTolerance));
    arm.setRotationDegrees(degrees - angleOffset);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setPercentOutput(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return (Arm.ticksToDegrees(arm.getEncoderPosition()) < (errorTolerance + degrees) && Arm.ticksToDegrees(arm.getEncoderPosition()) > (-errorTolerance + degrees));
  }
}
