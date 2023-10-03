// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto.pathplanner;

import org.team2168.commands.ScoringPositions.MidNode;
import org.team2168.commands.Wrist.OpenThenCloseWrist;
import org.team2168.commands.drivetrain.AdjustOnChargeStation;
import org.team2168.subsystems.Arm;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Elevator;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Turret;
import org.team2168.subsystems.WNE_Wrist;
import org.team2168.utils.PathUtil;
import org.team2168.utils.PathUtil.InitialPathState;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CrossOverChargingStationThenBalance extends SequentialCommandGroup {
  Paths paths;
  Drivetrain drivetrain;
  Elevator elevator;
  Arm arm;
  Turret turret;
  Limelight limelight;
  WNE_Wrist wrist;

  public CrossOverChargingStationThenBalance(Drivetrain drivetrain, WNE_Wrist wrist) {
    this.paths = Paths.getInstance();
    this.drivetrain = drivetrain;
    this.wrist = wrist;
    addCommands(PathUtil.getPathCommand(paths.crossOverChargingStation, drivetrain, InitialPathState.DISCARDHEADING),
        new MidNode(elevator, arm, turret, limelight), new OpenThenCloseWrist(null), new WaitCommand(0.4),
        PathUtil.getPathCommand(paths.toChargingStation, drivetrain, InitialPathState.PRESERVEODOMETRY),
        new WaitCommand(0.3), new AdjustOnChargeStation(drivetrain));
  }
}
