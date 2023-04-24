// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class WNE_Wrist extends SubsystemBase implements Loggable {
  /** Creates a new WNE_Arm. */
  private static DoubleSolenoid wristSolenoid;
  private static WNE_Wrist instance = null;

  public static WNE_Wrist getInstance() {
    if(instance == null)
      instance = new WNE_Wrist();
    return instance;
  }
  private WNE_Wrist() {
    wristSolenoid = new DoubleSolenoid(
                    PneumaticsModuleType.REVPH, 
                    Constants.PneumaticDevices.WIRST_FORWARD_CHANNEL, 
                    Constants.PneumaticDevices.WIRST_REVERSE_CHANNEL);
  }

  public void toggleWrist() {
    wristSolenoid.toggle();
  }

  /**
   * Sets the state of the solenoid
   * @param value the DoubleSolenoid.Value to set the wrist to (kOff, kForward, kReverse)
   */
  public void setWrist(DoubleSolenoid.Value value) {
    wristSolenoid.set(value);
  }

  public DoubleSolenoid.Value getWristState() {
    return wristSolenoid.get();
  }

  public void extend() {
    wristSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retract() {
    wristSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  @Log
  public boolean isExtended() {
    return wristSolenoid.get() == DoubleSolenoid.Value.kForward;
  }
  @Log
  public boolean isRetracted() {
    return wristSolenoid.get() == DoubleSolenoid.Value.kReverse;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
