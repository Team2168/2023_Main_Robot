// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Arm;

import java.util.function.DoubleSupplier;

import org.team2168.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveArmWithJoystick extends CommandBase {
  /** Creates a new DriveArmWithJoystick. */
  Arm arm;
  DoubleSupplier speed;
  public DriveArmWithJoystick(Arm arm, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.speed = speed;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setPercentOutput(speed.getAsDouble());
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
  }
}
