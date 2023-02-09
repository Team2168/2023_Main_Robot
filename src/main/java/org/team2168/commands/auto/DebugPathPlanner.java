// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import java.io.IOException;

import org.team2168.Constants;
import org.team2168.subsystems.Drivetrain;
import org.team2168.utils.PathUtil;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class DebugPathPlanner extends CommandBase {
  /** Creates a new DebugPathPlanner. */
  Drivetrain drivetrain;
  RamseteCommand rCommand;
  String pathname;

  public DebugPathPlanner(Drivetrain drivetrain, String pathname) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.pathname = pathname;

    // try {
    //   var trajectory = PathUtil.getPathPlannerTrajectory(pathname, true);

    //   rCommand = new RamseteCommand(
    //     trajectory,
    //     drivetrain::getPose,
    //     new RamseteController(Constants.Drivetrain.kRamseteB, Constants.Drivetrain.kRamseteZeta),
    //   )
    // } catch(IOException e) {
      
    // }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
