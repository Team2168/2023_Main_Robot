// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.HandCommands;

import org.team2168.Constants;
import org.team2168.subsystems.HandPneumatic;
import org.team2168.subsystems.HandWheels;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReverseIntake extends ParallelCommandGroup {
  /** Creates a new ReverseIntake. */
  /**Both reverses the intake and opens the claw with the pneumatic 
   * 
   * @param hand
   * @param handPneumatic
   */
  public ReverseIntake(HandWheels hand, HandPneumatic handPneumatic) {
    
    addCommands(new RunIntake(hand, Constants.MotorSpeeds.REVERSE_INTAKE_VELOCITY), new OpenIntake(handPneumatic));
  }
/** Only reverses the intake without actuating the pneumatic; the speed value is clamped between -1000.0 and 0.0, be sure it doesn't cross any of these values.
 * 
 * @param hand
 * @param speedValue
 */
  public ReverseIntake(HandWheels hand, double speedValue){
    addCommands(new RunIntake(hand, MathUtil.clamp(speedValue, -1000.0, 0.0)));
  }
}
