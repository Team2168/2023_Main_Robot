// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import org.team2168.Constants.OperatorConstants;
import org.team2168.commands.Autos;
import org.team2168.commands.DriveElevator;
import org.team2168.commands.DriveElevatorToPosition;
import org.team2168.commands.DriveElevatorToZero;
import org.team2168.commands.ExampleCommand;
import org.team2168.commands.Turret.*;
import org.team2168.subsystems.ExampleSubsystem;
import org.team2168.subsystems.Turret;
import org.team2168.OI;
import org.team2168.Constants.Joysticks;

import edu.wpi.first.wpilibj.Joystick;
import org.team2168.subsystems.Limelight;
import org.team2168.utils.F310;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import io.github.oblarg.oblog.Logger;

import org.team2168.subsystems.Elevator;
import org.team2168.utils.F310;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Turret turret = Turret.getInstance();
  private final OI oi = OI.getInstance();
  private final Elevator elevator = new Elevator();
  //public final F310 testJoystick = new F310(Joysticks.PID_TEST_JOYSTICK);
  static RobotContainer instance = null;
  private final Limelight limelight = Limelight.getInstance();
 


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public static RobotContainer getInstance() {
    if (instance == null){
          instance = new RobotContainer();
      }
      return instance;
   }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    Logger.configureLoggingAndConfig(this, false);
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    //elevator.setDefaultCommand(new DriveElevator(elevator, oi::getTestJoystickX)); //JOYSTICK USAGE
    elevator.setDefaultCommand(new DriveElevator(elevator, oi::getLeftOperatorJoystickY));

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
    oi.operatorJoystick.ButtonA().toggleOnTrue(new SetTurretToAngle(turret, 25.0));
    oi.operatorJoystick.ButtonB().toggleOnTrue(new ZeroTurret(turret));

    oi.operatorJoystick.ButtonRightStick().toggleOnTrue(new DriveTurretWithJoystick(turret, oi::getRightOperatorJoystickX));

  
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    
    oi.testJoystick.ButtonA().onTrue(new DriveElevatorToPosition(elevator, Constants.FieldMetrics.TOP_CONE_NODE_HEIGHT_IN, 5));
    oi.testJoystick.ButtonB().onTrue(new DriveElevatorToZero(elevator));
    oi.testJoystick.ButtonX().onTrue(new DriveElevatorToPosition(elevator, Constants.FieldMetrics.MIDDLE_CONE_NODE_HEIGHT_IN, 5));
    //oi.testJoystick.ButtonY().onTrue(new DriveElevator(elevator, 0.7));
 
    // m_driverController.rightBumper().onFalse(new ClampAndStopIntake(hand));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
