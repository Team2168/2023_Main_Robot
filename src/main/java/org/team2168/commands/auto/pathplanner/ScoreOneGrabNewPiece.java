// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto.pathplanner;

import org.team2168.commands.drivetrain.ResetOdometry;
import org.team2168.subsystems.Drivetrain;
import org.team2168.utils.PathUtil;
import org.team2168.utils.PathUtil.InitialPathState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreOneGrabNewPiece extends SequentialCommandGroup {
  /** Creates a new ScoreOneGrabNewPiece. */
  Drivetrain drivetrain;
  Paths paths = Paths.getInstance();
  
  public ScoreOneGrabNewPiece(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
  if (DriverStation.getAlliance() == Alliance.Red) {
    addCommands(new ResetOdometry(drivetrain, new Pose2d(13.75, 4.91, new Rotation2d(Units.degreesToRadians(0.0)))),
    PathUtil.getPathCommand(paths.path_score_leave_red_1, drivetrain, PathUtil.InitialPathState.PRESERVEODOMETRY),
    // should score here
    PathUtil.getPathCommand(paths.path_score_leave_red_2, drivetrain, PathUtil.InitialPathState.PRESERVEHEADING),
    // should pick up a game piece
    PathUtil.getPathCommand(paths.path_score_leave_red_3, drivetrain, InitialPathState.PRESERVEHEADING)
    //should score here
    );

  }
  else {
    addCommands(new ResetOdometry(drivetrain, new Pose2d(2.78, 4.90, new Rotation2d(Units.degreesToRadians(180.0)))),
    PathUtil.getPathCommand(paths.path_score_leave_blue_1, drivetrain, PathUtil.InitialPathState.PRESERVEODOMETRY),
    // should score here
    PathUtil.getPathCommand(paths.path_score_leave_blue_2, drivetrain, PathUtil.InitialPathState.PRESERVEHEADING),
    // should pick up a game piece
    PathUtil.getPathCommand(paths.path_score_leave_blue_3, drivetrain, InitialPathState.PRESERVEHEADING)
    //should score here
    );
  }
  }
}
