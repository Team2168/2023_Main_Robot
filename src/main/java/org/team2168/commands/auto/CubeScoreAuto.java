// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import org.team2168.commands.ScoringPositions.MidNode;
import org.team2168.commands.ScoringPositions.ReturnToFramePerimeter;
import org.team2168.commands.Turret.DriveTurretWithLimelight;
import org.team2168.commands.Wrist.CloseWrist;
import org.team2168.commands.Wrist.OpenWrist;
import org.team2168.commands.auto.pathplanner.Paths;
import org.team2168.commands.drivetrain.AdjustOnChargeStation;
import org.team2168.subsystems.Arm;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Elevator;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Turret;
import org.team2168.subsystems.WNE_Wrist;
import org.team2168.utils.PathUtil;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubeScoreAuto extends SequentialCommandGroup {
  // this should contain correction for trajectories using vision pose estimate,
  // automatic alignment, and maybe a holonomic drive controller
  public CubeScoreAuto(Drivetrain drive, Arm arm, Elevator elevator, Limelight lime, Turret turret, WNE_Wrist wrist) {
    Paths path = Paths.getInstance();

    addCommands(
        new ReturnToFramePerimeter(elevator, arm, turret, lime),
        raceWith(
            new DriveTurretWithLimelight(turret, lime),
            raceWith(
                PathUtil.getPathCommand(path.cube_node_path, drive, PathUtil.InitialPathState.DISCARDHEADING),
                new MidNode(elevator, arm, turret, lime)),
            new OpenWrist(wrist),
            new CloseWrist(wrist).withTimeout(0.5),
            raceWith(
                PathUtil.getPathCommand(path.return_charge_station, drive, PathUtil.InitialPathState.PRESERVEODOMETRY),
                new ReturnToFramePerimeter(elevator, arm, turret, lime)),
            new AdjustOnChargeStation(drive)));
  }
}
