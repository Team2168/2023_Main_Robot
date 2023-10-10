// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import edu.wpi.first.wpilibj.DriverStation;

import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.WNE_Wrist;
import org.team2168.subsystems.Wrist;
import org.team2168.subsystems.LEDs;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.github.oblarg.oblog.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Limelight limelight;
  private static Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  private Drivetrain drivetrain;
  private LEDs leds;
  private WNE_Wrist wrist;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = RobotContainer.getInstance();
    limelight = Limelight.getInstance();
    leds = LEDs.getInstance();
    wrist = WNE_Wrist.getInstance();
    compressor.enableDigital();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    // if (limelight.getCurrentPipeline() == 3) {
    //   leds.setLED(true, true, false);
    // } else if (limelight.getCurrentPipeline() == 4) {
    //   leds.setLED(true, false, true);
    // } else {
    // }
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    Logger.updateEntries();
  }

  /** This function is called once each ti
   * me the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    //makes Drivetrain able to be pushed only when the field is not real
    m_robotContainer.elevator.extendLock();
    if (!DriverStation.isFMSAttached()) {
      m_robotContainer.drivetrain.setMotorsCoast();
    }
    else {
    m_robotContainer.drivetrain.setMotorsBrake();
    }
    m_robotContainer.drivetrain.zeroHeading();
    m_robotContainer.elevator.extendLock();
  }
//\[]


  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.drivetrain.setMotorsBrakeAutos();
    m_robotContainer.elevator.retractLock();
    limelight.setPipeline(1);
    limelight.setLedMode(0);
    wrist.extend();
    // schedule the autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    limelight.enableBaseCameraSettings();
    limelight.setPipeline(1);
    limelight.setLedMode(0);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    wrist.extend();
    compressor.enableDigital();
    m_robotContainer.elevator.retractLock();
    m_robotContainer.drivetrain.setMotorsBrake();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
