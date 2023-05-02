// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;
import org.team2168.Constants.OperatorConstants;
import org.team2168.commands.ArmAndElevator;
import org.team2168.commands.Autos;
import org.team2168.commands.elevator.DriveElevator;
import org.team2168.commands.ExampleCommand;
import org.team2168.commands.led.*;
import org.team2168.subsystems.ExampleSubsystem;
import org.team2168.subsystems.LEDs;


import org.team2168.commands.Arm.BumpArm;
import org.team2168.commands.Arm.DriveArmWithJoystick;
import org.team2168.commands.Arm.RotateArm;
import org.team2168.commands.Limelight.SetPipeline;
import org.team2168.commands.ScoringPositions.GroundIntake;
import org.team2168.commands.ScoringPositions.HPStationIntake;
import org.team2168.commands.ScoringPositions.MidNode;
import org.team2168.commands.ScoringPositions.ReturnToFramePerimeter;
import org.team2168.commands.ScoringPositions.StowGamePieces;
import org.team2168.commands.auto.DoNothing;
import org.team2168.commands.auto.DriveForward;
import org.team2168.commands.auto.LeftLeaveCommunity;
import org.team2168.commands.auto.MidCS;
import org.team2168.commands.auto.ScoreLowHardCode;
import org.team2168.commands.auto.pathplanner.FourMetersPathplanner;
import org.team2168.commands.auto.pathplanner.ScoreLow;
import org.team2168.commands.drivetrain.AdjustOnChargeStation;
import org.team2168.commands.drivetrain.ArcadeDrive;
import org.team2168.commands.drivetrain.ToggleBrakes;
import org.team2168.commands.Wrist.CloseWrist;
import org.team2168.commands.Wrist.OpenWrist;
import org.team2168.commands.Wrist.ToggleWrist;
import org.team2168.subsystems.Arm;
import org.team2168.commands.elevator.DriveElevatorToPosition;
import org.team2168.commands.elevator.DriveElevatorToZero;
import org.team2168.commands.elevator.ExtendLock;
import org.team2168.commands.elevator.RetractLock;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Elevator;
import org.team2168.commands.Turret.*;
import org.team2168.subsystems.ExampleSubsystem;
import org.team2168.subsystems.Turret;
import org.team2168.OI;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.WNE_Wrist;
import org.team2168.subsystems.Wrist;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team2168.utils.F310;

import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  static RobotContainer instance = null;
  public final Elevator elevator = Elevator.getInstance();
  public final Drivetrain drivetrain = Drivetrain.getInstance();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final LEDs leds = LEDs.getInstance();

  private final Turret turret = Turret.getInstance();
  

  OI oi = OI.getInstance();
  
  //public final F310 testJoystick = new F310(Joysticks.PID_TEST_JOYSTICK);
  private final Limelight limelight = Limelight.getInstance();
  private final Arm arm = Arm.getInstance();
  private final WNE_Wrist wrist = WNE_Wrist.getInstance();

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
    autoChooser.addOption("Drive forward", new DriveForward(drivetrain));
    autoChooser.addOption("Score Low", new ScoreLow(drivetrain));
    autoChooser.addOption("SL Hard Code", new ScoreLowHardCode(drivetrain));

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


    // leds.setDefaultCommand(new SetEachLED(leds, false, false, false));

    // oi.testJoystick.ButtonA().onTrue(new SetEachLED(leds, false, false, true));
    // oi.testJoystick.ButtonB().onTrue(new SetEachLED(leds, true, false, false));
    // oi.testJoystick.ButtonX().onTrue(new SetEachLED(leds, true, true, true));
    // oi.testJoystick.ButtonY().onTrue(new SetEachLED(leds, true, false, true));



    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
 
    // m_driverController.rightBumper().onFalse(new ClampAndStopIntake(hand));

    //elevator.setDefaultCommand(new DriveElevator(elevator, oi::getTestJoystickX)); //JOYSTICK USAGE
    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, oi::getGunStyleTrigger, oi::getGunStyleWheel));
    elevator.setDefaultCommand(new DriveElevator(elevator, oi::getLeftOperatorJoystickY));
    turret.setDefaultCommand(new DriveTurretWithJoystick(turret, oi::getRightOperatorJoystickX));


    // if (limelight.getCurrentPipeline() == 3) {
    //   leds.setDefaultCommand(new TurnPurpleOn(leds, true, true, false));
    // } else if (limelight.getCurrentPipeline() == 4) {
    //   leds.setDefaultCommand(new TurnYellowOn(leds, true, false, true));
    // } else {
    //   leds.setDefaultCommand(new LEDRainbow(leds, 0));
    // }
    
    leds.setDefaultCommand(new LEDRainbow(leds, limelight, 0));

    //elevator.setDefaultCommand(new DriveElevator(elevator, oi::getTestJoystickX)); //JOYSTICK USAGE

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    // oi.driverJoystick.ButtonA().onTrue(new AdjustOnChargeStation(drivetrain));

    // oi.driverJoystick.ButtonLeftStick().onTrue(new DriveTurretWithLimelight(turret, limelight));
    // oi.driverJoystick.ButtonLeftStick().onTrue(new ToggleBrakes(drivetrain));
    oi.driverJoystick.ButtonLeftBumper().onTrue(new ZeroTurret(turret, limelight));
    oi.driverJoystick.ButtonA().onTrue(new SetPipeline(limelight, Limelight.Pipeline.REFLECTIVE_TAPE.pipelineValue));
    oi.driverJoystick.ButtonB().onTrue(new SetPipeline(limelight, Limelight.Pipeline.APRIL_TAGS.pipelineValue));
    oi.driverJoystick.ButtonX().onTrue(new SetPipeline(limelight, Limelight.Pipeline.SCAN_FOR_CONE.pipelineValue));
    oi.driverJoystick.ButtonY().onTrue(new SetPipeline(limelight, Limelight.Pipeline.SCAN_FOR_CUBE.pipelineValue));
    oi.driverJoystick.ButtonLeftStick().onTrue(new DriveTurretWithLimelight(turret, limelight));

    oi.operatorJoystick.ButtonLeftBumper().onTrue(new ReturnToFramePerimeter(elevator, arm, turret, limelight));
    oi.operatorJoystick.ButtonRightBumper().onTrue(new ToggleWrist(wrist));

    oi.operatorJoystick.ButtonA().onTrue(new GroundIntake(elevator, arm));
    oi.operatorJoystick.ButtonY().onTrue(new StowGamePieces(elevator, arm));
    oi.operatorJoystick.ButtonX().onTrue(new HPStationIntake(elevator, arm));
    oi.operatorJoystick.ButtonB().onTrue(new MidNode(elevator, arm, turret, limelight));

   
    oi.operatorJoystick.ButtonStart().onTrue(new ToggleWrist(wrist));
    
    oi.operatorJoystick.ButtonA().toggleOnTrue(new DriveElevatorToPosition(elevator, 0));


    oi.operatorJoystick.ButtonRightStick().onTrue(new DriveArmWithJoystick(arm, oi::getRightOperatorJoystickY));
    oi.operatorJoystick.ButtonLeftStick().onTrue(new DriveElevator(elevator, oi::getLeftOperatorJoystickY));
    oi.operatorJoystick.ButtonRightTrigger().onTrue(new DriveTurret(turret, 0.1));
    oi.operatorJoystick.ButtonLeftTrigger().onTrue(new DriveTurret(turret, -0.1));

    oi.testJoystick.ButtonA().onTrue(new ToggleWrist(wrist));
    oi.testJoystick.ButtonB().onTrue(new SetPipeline(limelight, Limelight.Pipeline.SCAN_FOR_CUBE.pipelineValue));
    oi.testJoystick.ButtonX().onTrue(new SetPipeline(limelight, Limelight.Pipeline.APRIL_TAGS.pipelineValue));

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
