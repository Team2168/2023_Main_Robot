// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto.pathplanner;

import org.team2168.commands.drivetrain.ArcadeDrive;
import org.team2168.subsystems.Drivetrain;
import org.team2168.utils.PathUtil;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreLow extends SequentialCommandGroup {
  /** Creates a new ScoreLow. */
  Drivetrain drivetrain;
  public ScoreLow(Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands(PathUtil.getPathPlannerCommand("score_low_wne_1", drivetrain, PathUtil.InitialPathState.DISCARDHEADING),
    addCommands(new ArcadeDrive(drivetrain, () -> 0.3, () -> 0.0).withTimeout(0.2),
    new WaitCommand(0.5),
    // PathUtil.getPathPlannerCommand("score_low_wne_2", drivetrain, PathUtil.InitialPathState.DISCARDHEADING));
    new ArcadeDrive(drivetrain, () -> -0.3, () -> 0.0).withTimeout(1.4));
  }
}
