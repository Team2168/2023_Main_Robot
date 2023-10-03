// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Wrist;

import org.team2168.subsystems.WNE_Wrist;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OpenThenCloseWrist extends SequentialCommandGroup {
  WNE_Wrist wrist;
  public OpenThenCloseWrist(WNE_Wrist wrist) {
    this.wrist = wrist;
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new OpenWrist(wrist).andThen(new CloseWrist(wrist)));
  }
}
