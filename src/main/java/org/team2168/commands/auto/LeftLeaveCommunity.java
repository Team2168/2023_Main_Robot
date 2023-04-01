// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import org.team2168.subsystems.Drivetrain;
import org.team2168.commands.auto.pathplanner.Paths;
import org.team2168.utils.PathUtil;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeftLeaveCommunity extends SequentialCommandGroup {

  public LeftLeaveCommunity(Drivetrain d) {

    Drivetrain drivetrain = d;

    Paths paths = Paths.getInstance();
    DriverStation.Alliance driverstationColor = DriverStation.getAlliance();

    Trajectory pathToUse;

    if(driverstationColor == DriverStation.Alliance.Red)
      pathToUse = paths.path_left_leave_community_red;
    else
      pathToUse = paths.path_left_leave_community_blue;

    addCommands(
      PathUtil.getPathCommand(pathToUse, drivetrain, PathUtil.InitialPathState.DISCARDHEADING)
    );
  }
}
