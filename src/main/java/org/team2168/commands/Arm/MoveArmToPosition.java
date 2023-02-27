// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Arm;

import org.team2168.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveArmToPosition extends CommandBase {
  /** Creates a new MoveArmToPosition. */
  private Arm arm;
  private double inches;
  private double errorTolerance;

  public MoveArmToPosition(Arm arm, double inches) {
    this(arm, inches, 1.0);
  }

  public MoveArmToPosition(Arm arm, double inches, double errorTolerance) {
    this.arm = arm;
    this.inches = inches;
    this.errorTolerance = errorTolerance;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var demand = Arm.inchesToDegrees(inches);
    arm.setRotationDegrees(demand);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (arm.getControllerError() < errorTolerance);
  }
}
