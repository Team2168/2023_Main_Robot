// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import org.team2168.Constants.Joysticks;
import org.team2168.Constants.OperatorConstants;
import org.team2168.commands.Autos;
import org.team2168.commands.elevator.DriveElevator;
import org.team2168.commands.ExampleCommand;
import org.team2168.commands.auto.DoNothing;
import org.team2168.commands.auto.LeftLeaveCommunity;
import org.team2168.commands.auto.MidCS;
import org.team2168.commands.auto.pathplanner.FourMetersPathplanner;
import org.team2168.commands.drivetrain.AdjustOnChargeStation;
import org.team2168.commands.drivetrain.ArcadeDrive;
import org.team2168.commands.drivetrain.ToggleBrakes;
import org.team2168.commands.elevator.DriveElevatorToPosition;
import org.team2168.commands.elevator.DriveElevatorToZero;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.ExampleSubsystem;
import org.team2168.Constants.Joysticks;

import edu.wpi.first.wpilibj.Joystick;
import org.team2168.subsystems.Limelight;
import org.team2168.utils.F310;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public final Drivetrain drivetrain = Drivetrain.getInstance();
  private final Elevator elevator = new Elevator();

  OI oi = OI.getInstance();
  
  //public final F310 testJoystick = new F310(Joysticks.PID_TEST_JOYSTICK);

  static RobotContainer instance = null;
  private final Limelight limelight = Limelight.getInstance();

  @Log(name = "Auto Chooser", width = 2)
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

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
    configureAutoRoutines();
  }

  public void configureAutoRoutines() {
    autoChooser.setDefaultOption("do nothing", new DoNothing());
    autoChooser.addOption("Left community", new LeftLeaveCommunity(drivetrain));
    autoChooser.addOption("Middle", new MidCS(drivetrain));
    autoChooser.addOption("4 m forward", new FourMetersPathplanner(drivetrain));

    SmartDashboard.putData(autoChooser);
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

    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, oi::getGunStyleTrigger, oi::getGunStyleWheel));


    //elevator.setDefaultCommand(new DriveElevator(elevator, oi::getTestJoystickX)); //JOYSTICK USAGE
    elevator.setDefaultCommand(new DriveElevator(elevator, oi::getOperatorJoystickY));

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    oi.driverJoystick.ButtonA().onTrue(new AdjustOnChargeStation(drivetrain));


    oi.driverJoystick.ButtonB().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    oi.driverJoystick.ButtonLeftBumper().onTrue(new ToggleBrakes(drivetrain));
    
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
    var auto = autoChooser.getSelected();
    if (auto == null) {
      System.out.println("selected path is null!");
      return new DoNothing();
    }
    else {
      return autoChooser.getSelected();
    }
  }
}
