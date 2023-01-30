// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HandPneumatic extends SubsystemBase {

  private DoubleSolenoid intakePneumatic;
  private static HandPneumatic instance = null;

  public HandPneumatic() {
    intakePneumatic = new DoubleSolenoid(Constants.PneumaticsModules.MODULE_TYPE,
    Constants.PneumaticsModules.INTAKE_CLAMP, Constants.PneumaticsModules.INTAKE_OPEN);
  }

  public static HandPneumatic getInstance(){
    if(instance == null){
      instance = new HandPneumatic();
    }
    return instance;
  }

  public void setClamp() {
    intakePneumatic.set(Value.kReverse);
  }

  public void setOpen() {
    intakePneumatic.set(Value.kForward);
  }

  public boolean isIntakeClamped(){
    return intakePneumatic.get() == Value.kReverse;
  }

  public boolean isIntakeOpen(){
    return intakePneumatic.get() == Value.kForward;
  }

  public boolean isIntakePneumaticOff(){
    return intakePneumatic.get() == Value.kOff;
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
