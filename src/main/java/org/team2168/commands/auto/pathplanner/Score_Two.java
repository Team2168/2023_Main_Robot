// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto.pathplanner;

import org.team2168.subsystems.Drivetrain;
import org.team2168.utils.PathUtil;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Score_Two extends CommandBase {
  /** Creates a new Score_Two. */
  public class EngageChargingStation extends SequentialCommandGroup {
    /** Creates a new EngageChargingStation. */
    public EngageChargingStation(Drivetrain d) {
      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());
      Drivetrain drivetrain = d;
  
      Paths paths = Paths.getInstance();
      //DriverStation.Alliance driverstationColor = DriverStation.getAlliance(); //meant for blue alliance not red alliance
  
      addCommands(PathUtil.getPathCommand(paths.path_score_two_nodes, drivetrain, PathUtil.InitialPathState.DISCARDHEADING));
    }
  }
}
