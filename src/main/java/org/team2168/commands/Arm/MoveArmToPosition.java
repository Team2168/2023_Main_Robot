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
  private boolean toFaceDown;
  private double errorTolerance;

  /**
   * Moves the arm a certain position
   * @param arm the Arm subsystem
   * @param inches the amount of inches for the arm to extend outwards
   */
  public MoveArmToPosition(Arm arm, double inches, boolean toFaceDown) {
    this(arm, inches, toFaceDown, 1.0);
  }

  public MoveArmToPosition(Arm arm, double inches, boolean toFaceDown, double errorTolerance) {
    this.arm = arm;
    this.inches = inches;
    this.toFaceDown = toFaceDown;
    this.errorTolerance = errorTolerance;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var demand = Arm.calculateDegreesfromInches(inches); //gives degrees to move as positive/clockwise position from 0 to 90 degrees
    if(toFaceDown)
      /* if the arm should be angled downwards, DEMAND becomes inverted so that it is within the arm's 
      range of motion from 0 to -90 degrees */
      demand = -demand;
    else 
      /* if the arm should be angled upwards, DEMAND gets added on to -180 (which is the arm's physical upper bound) to make the 
      resulting angle be between -180 and -90 */
      demand = -180 + demand;
      
    arm.setRotationDegrees(demand);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (arm.getControllerError() < errorTolerance);
  }
}
