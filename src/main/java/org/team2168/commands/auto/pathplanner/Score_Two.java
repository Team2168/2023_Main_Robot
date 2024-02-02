// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto.pathplanner;

import org.team2168.commands.drivetrain.ArcadeDrive;
import org.team2168.subsystems.Drivetrain;
import org.team2168.utils.PathUtil;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Score_Two extends SequentialCommandGroup {
  /** Creates a new Score_Two. */
    /** Creates a new EngageChargingStation. */
    public Score_Two(Drivetrain d) {
      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());
      Drivetrain drivetrain = d;
  
      Paths paths = Paths.getInstance();
      //DriverStation.Alliance driverstationColor = DriverStation.getAlliance(); //meant for blue alliance not red alliance
  
      addCommands(PathUtil.getPathCommand(paths.path_score_two_nodes_part1, drivetrain, PathUtil.InitialPathState.DISCARDHEADING));
      addCommands(new ArcadeDrive(drivetrain, () -> 0.3, () -> 0.0).withTimeout(0.2),
      new WaitCommand(0.5),
      PathUtil.getPathCommand(paths.path_score_two_nodes_part2, drivetrain, PathUtil.InitialPathState.DISCARDHEADING),
      new ArcadeDrive(drivetrain, () -> -0.3, () -> 0.0).withTimeout(1.4));
  }
}
