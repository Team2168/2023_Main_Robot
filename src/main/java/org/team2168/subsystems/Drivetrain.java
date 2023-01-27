// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;
import org.team2168.Constants.CANDevices;
import org.team2168.utils.PigeonHelper;
import org.team2168.utils.TalonFXHelper;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private PigeonHelper pidgey;

  private TalonFXHelper leftMotor1;
  private TalonFXHelper leftMotor2;
  private TalonFXHelper rightMotor1;
  private TalonFXHelper rightMotor2;

  private DifferentialDriveOdometry odometry;

  private static Drivetrain instance = null;

  private static final boolean ENABLE_CURRENT_LIMIT = true;
  private static final double CONTINUOUS_CURRENT_LIMIT = 35; // amps
  private static final double TRIGGER_THRESHOLD_LIMIT = 40; // amp
  private static final double TRIGGER_THRESHOLD_TIME = 0.2; // s
  private final static double NEUTRALDEADBAND = 0.001;

  private SupplyCurrentLimitConfiguration talonCurrentLimit;

  /**
   * Config Objects for motor controllers
  */
  TalonFXConfiguration leftConfig = new TalonFXConfiguration();
  TalonFXConfiguration rightConfig = new TalonFXConfiguration();

  public static final double TICKS_PER_REV = 2048.0; // one event per edge on each quadrature channel
  public static final double TICKS_PER_100MS = TICKS_PER_REV / 10.0;
  public static final double GEAR_RATIO = (50.0/10.0) * (38.0/28.0);
  public static final double WHEEL_DIAMETER = 4.0;
  public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; // inches
  public static final double PIGEON_UNITS_PER_ROTATION = 8192.0;
  public static final double DEGREES_PER_REV = 360.0;
  public static final double PIGEON_UNITS_PER_DEGREE = PIGEON_UNITS_PER_ROTATION / 360;
  public static final double WHEEL_BASE = 24.0; // distance between wheels (width) in inches
  public static final int TIMEOUT = 30;  // 30ms

    private double setPointPosition_sensorUnits;
    private double setPointHeading_sensorUnits;
  
  /**
   * Invert Directions for Left and Right
   */
  TalonFXInvertType leftInvert = TalonFXInvertType.CounterClockwise; // Same as invert = "false"
  TalonFXInvertType rightInvert = TalonFXInvertType.Clockwise; // Same as invert = "true"

  public static Drivetrain getInstance() {
    if (instance == null)
        instance = new Drivetrain();
    return instance;
}
  
  public Drivetrain() {
 // Instantiate motor objects
  leftMotor1 = new TalonFXHelper(CANDevices.DRIVETRAIN_LEFT_MOTOR_1);
  leftMotor2 = new TalonFXHelper(CANDevices.DRIVETRAIN_LEFT_MOTOR_2);
  rightMotor1 = new TalonFXHelper(CANDevices.DRIVETRAIN_RIGHT_MOTOR_1);
  rightMotor2 = new TalonFXHelper(CANDevices.DRIVETRAIN_RIGHT_MOTOR_2);

  pidgey = new PigeonHelper(CANDevices.PIGEON_IMU);

  leftMotor1.configFactoryDefault();
  leftMotor2.configFactoryDefault();
  rightMotor1.configFactoryDefault();
  rightMotor2.configFactoryDefault();

  leftMotor1.configFollowerStatusFrameRates();
  leftMotor2.configFollowerStatusFrameRates();
  rightMotor1.configClosedLoopStatusFrameRates();
  rightMotor2.configFollowerStatusFrameRates();

  // Create a current limit
  talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
  CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

  // Add the current limit to the motor configuration object
  leftConfig.supplyCurrLimit = talonCurrentLimit;
  rightConfig.supplyCurrLimit = talonCurrentLimit;
  
  setMotorsBrake();

  pidgey.setReducedStatusFramePeriods();
  odometry = new DifferentialDriveOdometry(pidgey.getRotation2d());
  
  /* Set Neutral Mode */
  leftMotor1.setNeutralMode(NeutralMode.Brake);
  rightMotor1.setNeutralMode(NeutralMode.Brake);

  /* Configure output and sensor direction */
  leftMotor1.setInverted(leftInvert);
  leftMotor2.setInverted(leftInvert);
  rightMotor1.setInverted(rightInvert);
  rightMotor2.setInverted(rightInvert);

  /* Reset Pigeon Configs */
  pidgey.configFactoryDefault();
  pidgey.setReducedStatusFramePeriods();

  /** Feedback Sensor Configuration */

	/** Distance Configs */

	/* Configure the left Talon's selected sensor as integrated sensor */
	leftConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Feedback Source

  /* Configure the Remote (Left) Talon's selected sensor as a remote sensor for the right Talon */
  rightConfig.remoteFilter0.remoteSensorDeviceID = leftMotor1.getDeviceID(); //Device ID of Remote Source
  rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; //Remote Source Type

  /* Now that the Left sensor can be used by the master Talon,
	 * set up the Left (Aux) and Right (Master) distance into a single
	 * Robot distance as the Master's Selected Sensor 0. */
	setRobotDistanceConfigs(rightInvert, rightConfig);

		// /* FPID for Distance */
		// rightConfig.slot0.kF = Constants.Drivetrain.kGains_Distance.kF; // need new gains after we get the drivetrain
		// rightConfig.slot0.kP = Constants.Drivetrain.kGains_Distance.kP;
		// rightConfig.slot0.kI = Constants.Drivetrain.kGains_Distance.kI;
		// rightConfig.slot0.kD = Constants.Drivetrain.kGains_Distance.kD;
		// rightConfig.slot0.integralZone = Constants.Drivetrain.kGains_Distance.kIzone;
		// rightConfig.slot0.closedLoopPeakOutput = Constants.Drivetrain.kGains_Distance.kPeakOutput;

    /** Heading Configs */
		rightConfig.remoteFilter1.remoteSensorDeviceID = pidgey.getDeviceID();    //Pigeon Device ID
		rightConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.Pigeon_Yaw; //This is for a Pigeon over CAN
		rightConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice(); //Set as the Aux Sensor
		rightConfig.auxiliaryPID.selectedFeedbackCoefficient = 3600.0 / Constants.Drivetrain.kPigeonUnitsPerRotation; //Convert Yaw to tenths of a degree

		// /* FPID for Heading */
		// rightConfig.slot1.kF = Constants.Drivetrain.kGains_Turning.kF; // No gains for this yet
		// rightConfig.slot1.kP = Constants.Drivetrain.kGains_Turning.kP;
		// rightConfig.slot1.kI = Constants.Drivetrain.kGains_Turning.kI;
		// rightConfig.slot1.kD = Constants.Drivetrain.kGains_Turning.kD;
		// rightConfig.slot1.integralZone = Constants.Drivetrain.kGains_Turning.kIzone;
		// rightConfig.slot1.closedLoopPeakOutput = Constants.Drivetrain.kGains_Turning.kPeakOutput;
  
    leftConfig.neutralDeadband = NEUTRALDEADBAND;
		rightConfig.neutralDeadband = NEUTRALDEADBAND;

    /**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
		rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
		rightConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
		rightConfig.slot3.closedLoopPeriod = closedLoopTimeMs;

      // /* Motion Magic Configs */ // need new configs for 2023
      // rightConfig.motionAcceleration = (int) (inchesPerSecToTicksPer100ms(8.0*12.0)); //(distance units per 100 ms) per second
      // rightConfig.motionCruiseVelocity = (int) (inchesPerSecToTicksPer100ms(10.0*12.0));

  /* APPLY the config settings */
  leftMotor1.configAllSettings(leftConfig);
  leftMotor2.configAllSettings(leftConfig);
  rightMotor1.configAllSettings(rightConfig);
  rightMotor2.configAllSettings(rightConfig);

  /* Set status frame periods to ensure we don't have stale data */
		/* These aren't configs (they're not persistant) so we can set these after the configs.  */
	rightMotor1.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, TIMEOUT);
	rightMotor1.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, TIMEOUT);
	rightMotor1.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, TIMEOUT);
	rightMotor1.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10, TIMEOUT);
	leftMotor1.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, TIMEOUT);
	pidgey.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR , 5, TIMEOUT);        


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
