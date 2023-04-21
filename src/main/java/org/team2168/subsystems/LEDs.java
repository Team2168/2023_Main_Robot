// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants.PneumaticDevices;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class LEDs extends SubsystemBase {

  /*LED Objects are created here */

  private Solenoid redLED;
  private Solenoid blueLED;
  private Solenoid greenLED;
  private Solenoid redLEDTwo;
  private Solenoid blueLEDTwo;
  private Solenoid greenLEDTwo;

  static LEDs instance = null;

  /** Creates a constructor */
  public LEDs() {
    redLED = new Solenoid(PneumaticDevices.MODULE_TYPE, PneumaticDevices.RED_LED); //these are placeholders (see constants)
    blueLED = new Solenoid(PneumaticDevices.MODULE_TYPE, PneumaticDevices.BLUE_LED); //these are placeholders                 
    greenLED = new Solenoid(PneumaticDevices.MODULE_TYPE, PneumaticDevices.GREEN_LED); //these are placeholders
    redLEDTwo = new Solenoid(PneumaticDevices.MODULE_TYPE, PneumaticDevices.RED_LED_TWO); //these are placeholders (see constants)
    blueLEDTwo = new Solenoid(PneumaticDevices.MODULE_TYPE, PneumaticDevices.BLUE_LED_TWO); //these are placeholders                 
    greenLEDTwo = new Solenoid(PneumaticDevices.MODULE_TYPE, PneumaticDevices.GREEN_LED_TWO); //these are placeholders
  }

  //these commands turn on and off the different colored LEDs (if its true, the light will be on, if its false, the light will be off)
  //for yellow and purple, it combines the redOnOff, greenOnOff, and blueOnOff methods to create new colors (this may be phased out)

  public void redOnOff(boolean isOn){
    redLED.set(isOn);
    redLEDTwo.set(isOn);
  }

  public void blueOnOff(boolean isOn){
    blueLED.set(isOn);
    blueLEDTwo.set(isOn);
  }

  public void greenOnOff(boolean isOn){
    greenLED.set(isOn);
    greenLEDTwo.set(isOn);
  }

  public void yellowOnOff(boolean isOn){
    redOnOff(true);
    greenOnOff(true);
    blueOnOff(false);
  }

  public void purpleOnOff(boolean isOn){
    redOnOff(true);
    blueOnOff(true);
    greenOnOff(false);
  }


  @Config
  public void setLED(boolean redOn, boolean blueOn, boolean greenOn){
    redOnOff(redOn);
    blueOnOff(blueOn);
    greenOnOff(greenOn);
  }

  //these methods get the state of the LED and return it (i.e. it will tell you how it's doing)

  @Log()
  public boolean getRedState(){
    return redLED.get();
  }

  @Log()
  public boolean getGreenState(){
    return greenLED.get();
  }

  @Log()
  public boolean getBlueState(){
    return blueLED.get();
  }

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
