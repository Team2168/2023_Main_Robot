// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import org.team2168.Constants;
import org.team2168.subsystems.Arm;
import org.team2168.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmAndElevator extends CommandBase {
  private Arm arm;
  private Elevator elevator;
  private double positionX;
  private double positionY;
  private boolean armFaceDown;
  private double errorTolerance;

  /**
   * Commands the arm and elevator to move the wrist to be a certain height and extension from the robot
   * @param arm The arm subsystem
   * @param elevator The elevator system
   * @param positionX The amount in inches for the arm to be extended
   * @param positionY The height in inches for the wrist to be 
   */
  public ArmAndElevator(Arm arm, Elevator elevator, double positionX, double positionY, boolean armFaceDown) {
    this(arm, elevator, positionX, positionY, armFaceDown, 1.0);

    addRequirements(arm);
    addRequirements(elevator);
  }

  /**
   * Commands the arm and elevator to move the wrist to be a certain height and extension from the robot
   * @param arm The arm subsystem
   * @param elevator The elevator system
   * @param positionX The amount in inches for the arm to be extended
   * @param positionY The height in inches for the wrist to be 
   * @param errorTolerance The amount of allowable error in inches for the elevator and degrees for the arm
   */
  public ArmAndElevator(Arm arm, Elevator elevator, double positionX, double positionY, boolean ArmFaceDown, double errorTolerance) {
    this.arm = arm;
    this.elevator = elevator;
    this.positionX = positionX;
    this.positionY = positionY;
    this.armFaceDown = armFaceDown;
    this.errorTolerance = errorTolerance;

    addRequirements(arm);
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double degrees = arm.calculateDegreesfromInches(positionX);
    if(armFaceDown)
      /* if the arm should be angled downwards, DEMAND becomes inverted so that it is within the arm's 
      range of motion from 0 to -90 degrees */
      degrees = -degrees;
    else 
      /* if the arm should be angled upwards, DEMAND gets added on to -180 (which is the arm's physical upper bound) to make the 
      resulting angle be between -180 and -90 */
      degrees = -180 + degrees;
      
    double a = Constants.RobotMetrics.ARM_LENGTH * Math.cos(degrees); //the height the arm is at when at a certain position
    double h = positionY - a; //the to-be height of the elevator

    arm.setRotationDegrees(degrees);
    elevator.setPosition(h);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setSpeed(0);
    elevator.setSpeedVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.getControllerError() <= errorTolerance; //&& elevator.getControllerError() <= errorTolerance
  }
}
