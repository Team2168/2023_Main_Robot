// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import org.team2168.Constants;
import org.team2168.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveHandToPosition extends CommandBase {
  /** Creates a new MoveArmToPosition. */

  private Arm arm;
  //private Elevator elevator;
  private double x;
  // double y;
  private double errorTolerance; //both inches and degrees

  /**
   * How far out the arm should move 
   * @param arm the Arm subsystem
   * @param x how far out the arm should move (inches)
   */
  public MoveHandToPosition(Arm arm, double x) {
    this(arm, x, 1.0);

    addRequirements(arm);
  }

  public MoveHandToPosition(Arm arm, double x, double errorTolerance) {
    this.arm = arm;
    this.x = x;
    this.errorTolerance = errorTolerance;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double degreesToRotate = -Math.toDegrees(Math.asin(x/Constants.RobotMetrics.ARM_LENGTH));
    if(arm.getPositionDegrees() < -90)
      degreesToRotate = -degreesToRotate; //makes degreesToRotate positive
    arm.setRotationDegrees(degreesToRotate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (arm.getControllerError() < errorTolerance); //to later include && elevator.getError < errorTolerance
  }
}
