// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import org.team2168.commands.drivetrain.ArcadeDrive;
import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreLowHardCode extends SequentialCommandGroup {
  /** Creates a new ScoreLowHardCode. */
  Drivetrain drivetrain;
  public ScoreLowHardCode(Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.drivetrain = drivetrain;
    addCommands(new ArcadeDrive(drivetrain, () -> 0.5, () -> 0.0).withTimeout(0.4));
  }
}
