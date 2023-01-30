// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.HandCommands;

import org.team2168.Constants;
import org.team2168.subsystems.HandPneumatic;
import org.team2168.subsystems.HandWheels;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OpenAndRunIntake extends ParallelCommandGroup {
 
  public OpenAndRunIntake(HandWheels hand, HandPneumatic handPneumatic) {
    
    addCommands(
      new OpenIntake(handPneumatic),
  new RunIntake(hand, Constants.MotorSpeeds.FORWARD_INTAKE_VELOCITY)

    );
  }

  
}
