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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Drivetrain extends SubsystemBase implements Loggable {
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
  private final static double NEUTRAL_DEADBAND = 0.001;

  private SupplyCurrentLimitConfiguration talonCurrentLimit;

  /**
   * Config Objects for motor controllers
  */
  TalonFXConfiguration leftConfig = new TalonFXConfiguration();
  TalonFXConfiguration rightConfig = new TalonFXConfiguration();

  public static final double TICKS_PER_REV = 2048.0; // one event per edge on each quadrature channel
  public static final double TICKS_PER_100MS = TICKS_PER_REV / 10.0;
  public static final double GEAR_RATIO = (60.0/10.0) * (38.0/22.0);
  public static final double WHEEL_DIAMETER = 6.0;
  public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; // inches
  public static final double PIGEON_UNITS_PER_ROTATION = 8192.0;
  public static final double DEGREES_PER_REV = 360.0;
  public static final double PIGEON_UNITS_PER_DEGREE = PIGEON_UNITS_PER_ROTATION / 360;
  public static final double WHEEL_BASE = 23.0; // distance between wheels (width) in inches
  public static final int TIMEOUT = 30;  // 30ms

  public static final double kV = 0.0;
  public static final double kA = 0.0;

  private double setPointPosition_sensorUnits;
  private double setPointHeading_sensorUnits;

  private boolean areTheBrakesToBeBrakesEnabled;

  /**
   * Invert Directions for Left and Right
   */
  TalonFXInvertType leftInvert = TalonFXInvertType.Clockwise; // Same as invert = "true"
  TalonFXInvertType rightInvert = TalonFXInvertType.CounterClockwise; // Same as invert = "false"


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
  odometry = new DifferentialDriveOdometry(pidgey.getRotation2d(), 0.0, 0.0);
  
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
  
    leftConfig.neutralDeadband = NEUTRAL_DEADBAND;
		rightConfig.neutralDeadband = NEUTRAL_DEADBAND;

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
      rightConfig.motionAcceleration = (int) (inchesPerSecToTicksPer100ms(8.0*12.0)); //(distance units per 100 ms) per second
      rightConfig.motionCruiseVelocity = (int) (inchesPerSecToTicksPer100ms(10.0*12.0));

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
    Rotation2d rot;

    rot = pidgey.getRotation2d();
    odometry.update(rot, getLeftEncoderDistance(), getRightEncoderDistance());
  }

  public void teleopconfigs() {
    rightMotor1.configAllSettings(rightConfig);
    rightMotor1.configAllSettings(rightConfig);
    leftMotor1.configAllSettings(leftConfig);
    leftMotor2.configAllSettings(leftConfig);
  }

  @Log(name = "Is are get brakes brakes enabed", columnIndex = 1, rowIndex = 0)
  public boolean areTheBrakesToBeBrakesEnabled() {
    return areTheBrakesToBeBrakesEnabled;
  }

  @Config(name = "are the brakes to be engaged?", width = 1)
  public void setAreTheBrakesToBeBrakesEnabled(boolean toAmBrakesEnabled) {
    areTheBrakesToBeBrakesEnabled = toAmBrakesEnabled;
    if (toAmBrakesEnabled) {
        setMotorsBrake();
    } else {
        setMotorsCoast();
    }
  }

  /**
   * Gets the odometry pose
   *
   * @return Pose2d odometry pose
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * @param vel set the cruise velocity (in/sec)
   */
  public void setCruiseVelocity(double vel) {
    rightMotor1.configMotionCruiseVelocity((int) (inchesPerSecToTicksPer100ms(vel))); //distance units per 100 ms
  }

  /**
   * Gets gyro heading
   *
   * @return gyro heading from -180.0 to 180.0 degrees. Positive counterclockwise
   */
  public double getHeading() {
    return pidgey.getRotation2d().getDegrees();
  }

  /**
   * Gets gyro pitch
   *
   * @return gyro pitch degrees
   */
  @Log
  public double getPitch() {
    return pidgey.getPitch();
  }

  /**
   * Gets gyro turn rate
   *
   * @return rate in degrees per second
   */
  public double getTurnRate() {
    return -pidgey.getRate();
  }

  /**
   * Get average encoder distance
   *
   * @return gets distance in meters
   */
  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
  }

  /**
   * Gets average encoder distance
   * 
   * @return gets distance in inches
   */
  public double getAverageEncoderDistanceIn() {
    return Units.metersToInches((getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0);
  }

  /**
     * Gets wheel speeds in meters per second
     *
     * @return DifferentialDriveWheelSpeeds object
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
      return new DifferentialDriveWheelSpeeds(getLeftEncoderRate(), getRightEncoderRate());
  }

  /**
     * Gets left encoder distance in raw sensor units
     *
     * @return distance in sensor ticks
     */
    public double getLeftEncoderDistanceRaw() {
      return leftMotor1.getSelectedSensorPosition();
  }

  /**
   * Gets left encoder distance
   *
   * @return encoder distance in meters
   */
  public double getLeftEncoderDistance() {
    return ticksToMeters(getLeftEncoderDistanceRaw());
  }

  /**
   * Gets left encoder distance
   * 
   * @return encoder distance in inches
   */
  public double getLeftEncoderDistanceIn() {
    return Units.metersToInches(ticksToMeters(getLeftEncoderDistanceRaw()));
  }

  /**
   * Gets right encoder distance in raw sensor ticks
   *
   * @return distance in sensor ticks
   */
  public double getRightEncoderDistanceRaw() {
    return rightMotor1.getSelectedSensorPosition();
  }

  /**
   * Gets right encoder distance
   *
   * @return encoder distance in meters
   */
  public double getRightEncoderDistance() {
    return ticksToMeters(getRightEncoderDistanceRaw());
  }

  /**
   * Gets right encoder distance
   * 
   * @return encoder distance in inches
   */
  public double getRightEncoderDistanceIn() {
    return Units.metersToInches(getRightEncoderDistance());
  }

  /**
   * Zeroes gyro heading
   */
  public void zeroHeading() {
    pidgey.reset();
    pidgey.setYaw(0, TIMEOUT);
    pidgey.setAccumZAngle(0, TIMEOUT);
  }

  /**
   * Resets encoders on motors
   */
  public void resetEncoders() {
    leftMotor1.getSensorCollection().setIntegratedSensorPosition(0, TIMEOUT);
    rightMotor1.getSensorCollection().setIntegratedSensorPosition(0, TIMEOUT);

    leftMotor1.setSelectedSensorPosition(0.0);
    rightMotor1.setSelectedSensorPosition(0.0);
}

  /**
   * Resets odometry to specified pose
   *
   * @param pose            pose to set odometry
   * @param preserveHeading do we preserve the gyro heading?
   */

  public void resetOdometry(Pose2d pose, boolean preserveHeading) {
    Rotation2d rot;

    rot = pidgey.getRotation2d();

    resetEncoders();
        if (!preserveHeading)
            zeroHeading();
    odometry.resetPosition(rot, 0.0, 0.0, pose);
  }

  /**
     * Reset odometry to specified pose, while resetting the gyro.
     *
     * @param pose pose to set odometry
     */
    public void resetOdometry(Pose2d pose) {
      this.resetOdometry(pose, false);
  }

  /**
   * Gets Left encoder velocity
   *
   * @return velocity in sensor ticks
   */
  public double getLeftEncoderRateRaw() {
    return leftMotor1.getSelectedSensorVelocity();
  }

  /**
   * Gets Right encoder velocity
   *
   * @return velocity in sensor ticks
   */
  public double getRightEncoderRateRaw() {
    return rightMotor1.getSelectedSensorVelocity();
  }

  /**
   * Gets left encoder velocity
   *
   * @return encoder velocity in meters/second
   */
  public double getLeftEncoderRate() {
    return ticksToMeters(getLeftEncoderRateRaw()) * 10.0;
  }

  /**
   * Gets right encoder velocity
   *
   * @return encoder velocity in meters/second
   */
  public double getRightEncoderRate() {
    return ticksToMeters(getRightEncoderRateRaw()) * 10.0;
  }

  /**
   * Converts an input of inches to encoder ticks
   * @param setpoint inches to be converted into ticks
   * @return ticks from inches inputted
   */
  private double inchesToTicks(double setpoint) {
    return (setpoint * TICKS_PER_REV * GEAR_RATIO) / WHEEL_CIRCUMFERENCE;
  }

  /**
   * Converts input of inches per second to ticks per 100 ms
   * @param setpoint inches per second to be converted into ticks per 100 ms
   * @return ticks per second from inches inputted
   */
  private double inchesPerSecToTicksPer100ms(double setpoint) {
    return inchesToTicks(setpoint) / 10.0;
  }

  /** 
   * Converts input of encoder ticks to inches
   * @param setpoint encoder ticks to be converted into inches
   * @return inches converted from ticks inputted
   */
  private double ticksToInches(double setpoint) {
    return (setpoint * WHEEL_CIRCUMFERENCE) / (TICKS_PER_REV * GEAR_RATIO);
  }

  /**
   * Converts input of encoder ticks to meters
   * @param ticks encoder ticks to be converted into meters
   * @return meters converted from ticks inputted
   */
  private double ticksToMeters(double ticks) {
    // the cheezy poofs have been colluding to get inside of our codebase
    return ticksToInches(ticks) * 0.0254;
  }

  /**
   * Converts a setpoint in degrees to IMU 'encoder ticks'
   *
   * @param setpoint
   * @return
   */
  private double degreesToTicks(double setpoint) {
    // return (setpoint / DEGREES_PER_REV) * PIGEON_UNITS_PER_ROTATION / 2.0;
    return setpoint * 10.0;
  }

  /**
   * Creates a tank drive which commands individual wheel speeds for each side of the robot
   * @param leftSpeed speed for left wheels in terms of percentage of max output. Ranges from
   * -1.0 to 1.0.
   * @param rightSpeed speed for right wheels in terms of percentage of max output. Ranges from
   * -1.0 to 1.0.
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    // drive.tankDrive(leftSpeed, rightSpeed);
    leftMotor1.set(leftSpeed);
    leftMotor2.set(leftSpeed);
    rightMotor1.set(rightSpeed);
    rightMotor2.set(rightSpeed);
  }

  /**
   * Creates tank drive which commands individual voltage for each side of the robot
   * @param leftVolts voltage commanded to the left wheels
   * @param rightVolts voltage commanded to the right wheels
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    //TODO: Change to use voltage compensation built into the motor controllers?
    // double batteryVoltage = RobotController.getBatteryVoltage();
    tankDrive(leftVolts / Constants.Drivetrain.MAX_VOLTAGE, rightVolts / Constants.Drivetrain.MAX_VOLTAGE);
  }

  /**
   * Creates arcade drive which commands a drivetrain speed for both wheels, and commands a rotation speed which
   * is added/subtracted for both wheels, in opposite directions
   * @param xSpeed forward speed of the drivetrain, which is a percentage of the maximum output
   * @param zRotation percentage of output to add/subtract in opposite directions for each side of the drivetrain
   */
  public void arcadeDrive(double xSpeed, double zRotation) {
    tankDrive(xSpeed + zRotation, xSpeed - zRotation);
  }
  
  /**
   * Change all motors to their default mix of brake/coast modes.
   * Should be used for normal match play.
   */
  public void setMotorsBrake() {
    leftMotor1.setNeutralMode(NeutralMode.Brake);
    leftMotor2.setNeutralMode(NeutralMode.Coast);
    rightMotor1.setNeutralMode(NeutralMode.Brake);
    rightMotor2.setNeutralMode(NeutralMode.Coast);
    areTheBrakesToBeBrakesEnabled = true;
  }

  /** 
   * Change all motors to their brake settings for the autonomous period
   */
  public void setMotorsBrakeAutos() {
    leftMotor1.setNeutralMode(NeutralMode.Brake);
    leftMotor2.setNeutralMode(NeutralMode.Brake);
    rightMotor1.setNeutralMode(NeutralMode.Brake);
    rightMotor2.setNeutralMode(NeutralMode.Brake);
    areTheBrakesToBeBrakesEnabled = true;
  }

  /**
   * Change all the drivetrain motor controllers to coast mode.
   * Useful for allowing robot to be manually pushed around the field.
   */
  public void setMotorsCoast() {
    leftMotor1.setNeutralMode(NeutralMode.Coast);
    leftMotor2.setNeutralMode(NeutralMode.Coast);
    rightMotor1.setNeutralMode(NeutralMode.Coast);
    rightMotor2.setNeutralMode(NeutralMode.Coast);
    areTheBrakesToBeBrakesEnabled = false;
  }

 /** 
	 * Determines if SensorSum or SensorDiff should be used 
	 * for combining left/right sensors into Robot Distance.  
	 * 
	 * Assumes Aux Position is set as Remote Sensor 0.  
	 * 
	 * configAllSettings must still be called on the master config
	 * after this function modifies the config values. 
	 * 
	 * @param masterInvertType Invert of the Master Talon
	 * @param masterConfig Configuration object to fill
	 */
  void setRobotDistanceConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig){
		/**
		 * Determine if we need a Sum or Difference.
		 * 
		 * The auxiliary Talon FX will always be positive
		 * in the forward direction because it's a selected sensor
		 * over the CAN bus.
		 * 
		 * The master's native integrated sensor may not always be positive when forward because
		 * sensor phase is only applied to *Selected Sensors*, not native
		 * sensor sources.  And we need the native to be combined with the 
		 * aux (other side's) distance into a single robot distance.
		 */

		/* THIS FUNCTION should not need to be modified. 
		   This setup will work regardless of whether the master
		   is on the Right or Left side since it only deals with
		   distance magnitude.  */

		/* Check if we're inverted */
		if (masterInvertType == TalonFXInvertType.Clockwise){
			/* 
				If master is inverted, that means the integrated sensor
				will be negative in the forward direction.
				If master is inverted, the final sum/diff result will also be inverted.
				This is how Talon FX corrects the sensor phase when inverting 
				the motor direction.  This inversion applies to the *Selected Sensor*,
				not the native value.
				Will a sensor sum or difference give us a positive total magnitude?
				Remember the Master is one side of your drivetrain distance and 
				Auxiliary is the other side's distance.
					Phase | Term 0   |   Term 1  | Result
				Sum:  -((-)Master + (+)Aux   )| NOT OK, will cancel each other out
				Diff: -((-)Master - (+)Aux   )| OK - This is what we want, magnitude will be correct and positive.
				Diff: -((+)Aux    - (-)Master)| NOT OK, magnitude will be correct but negative
			*/

			masterConfig.diff0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Integrated Sensor
			masterConfig.diff1Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();   //Aux Selected Sensor
			masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); //Diff0 - Diff1
		} else {
			/* Master is not inverted, both sides are positive so we can sum them. */
			masterConfig.sum0Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();    //Aux Selected Sensor
			masterConfig.sum1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local IntegratedSensor
			masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); //Sum0 + Sum1
		}

		/* Since the Distance is the sum of the two sides, divide by 2 so the total isn't double
		   the real-world value */
		masterConfig.primaryPID.selectedFeedbackCoefficient = 0.5;
	 }
}
