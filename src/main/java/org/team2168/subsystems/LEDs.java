// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

  /*LED Objects are created here */

  private Solenoid redLED;
  //private Solenoid blueLED;
  private Solenoid greenLED;

  static LEDs instance = null;

  /** Creates a constructor */
  public LEDs() {
    redLED = new Solenoid(null, 0); //these are placeholders
    //blueLED = new Solenoid(null, 0);                  /* an extra LED is probably not needed as of now */
    greenLED = new Solenoid(null, 0); //these are placeholders
  }

  //these commands turn on and off the different colored LEDs (if its true, the light will be on, if its false, the light will be off)

  public void redOnOff(boolean isOn){
    redLED.set(isOn);
  }

 /*  public void blueOnOff(boolean isOn){
    blueLED.set(isOn);
  }
*/
  public void greenOnOff(boolean isOn){
    greenLED.set(isOn);
  }

  //these methods get the state of the LED and return it (i.e. it will tell you how it's doing)

  public boolean getRedState(){
    return redLED.get();
  }

  public boolean getGreenState(){
    return greenLED.get();
  }

  /*public boolean getBlueState(){
    return blueLED.get();
  }
*/

  public static LEDs getInstance(){
    if (instance == null){
      instance = new LEDs();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
