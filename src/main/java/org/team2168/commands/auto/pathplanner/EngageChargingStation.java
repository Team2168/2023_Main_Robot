// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto.pathplanner;

import org.team2168.subsystems.Drivetrain;
import org.team2168.utils.PathUtil;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EngageChargingStation extends SequentialCommandGroup {
  /** Creates a new EngageChargingStation. */
  public EngageChargingStation(Drivetrain d) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Drivetrain drivetrain = d;

    Paths paths = Paths.getInstance();
    //DriverStation.Alliance driverstationColor = DriverStation.getAlliance(); //meant for blue alliance not red alliance

    addCommands(PathUtil.getPathCommand(paths.engage_station, drivetrain, PathUtil.InitialPathState.DISCARDHEADING));
  }
}
