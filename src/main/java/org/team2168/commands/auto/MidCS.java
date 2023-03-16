// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import org.team2168.subsystems.Drivetrain;
import org.team2168.commands.drivetrain.AdjustOnChargeStation;
import org.team2168.commands.drivetrain.ArcadeDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidCS extends SequentialCommandGroup {
  /** Creates a new MidCSStation. */
  public MidCS(Drivetrain d) {

    Drivetrain drivetrain;
    
    drivetrain = d;

    addCommands(
      new ArcadeDrive(drivetrain, () -> {return -0.5;}, () -> {return 0.0;}).withTimeout(2.1),
      new DoNothing().withTimeout(2.0),
      new ArcadeDrive(drivetrain, () -> {return 0.5;}, () -> {return 0.0;}).withTimeout(1.3)//,
      // new AdjustOnChargeStation(drivetrain).withTimeout(7.0)
    );
  }
}
