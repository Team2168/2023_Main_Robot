// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;
import org.team2168.utils.TalonFXHelper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

  public enum ArmPosition{ //TODO: change all the 0.0s, they are placeholders
    AUTO_HIGH_CONE_NODE(0.0), //see if auto positions can be the same as the teleop positions
    AUTO_HIGH_CUBE_NODE(0.0),
    AUTO_MID_CONE_NODE(0.0),
    AUTO_MID_CUBE_NODE(0.0),
    AUTO_HYBRID_NODE(0.0),
    AUTO_STAGING_MARK(0.0),
    HIGH_CONE_NODE(0.0),
    HIGH_CUBE_NODE(0.0),
    MID_CONE_NODE(0.0),
    MID_CUBE_NODE(0.0),
    HYBRID_NODE(0.0),
    STAGING_MARK(0.0),
    HP_STATION(0.0),
    GROUND(0.0); //I don't know if we'll be able to pick up stuff from the ground

    public final double position_degrees;

    private ArmPosition(double position_degrees){
      this.position_degrees = position_degrees;
    }
  }

  private static TalonFXHelper armMotor; 
  private static Arm instance = null;

  private static SingleJointedArmSim armSim;
  private static TalonFXSimCollection armMotorSim;

  private static double setpoint = 0.0;

  // Current limit configuration
  private SupplyCurrentLimitConfiguration talonCurrentLimit;
  private static final boolean ENABLE_CURRENT_LIMIT = true;
  private static final double CONTINUOUS_CURRENT_LIMIT = 20; // amps TODO: update limit
  private static final double TRIGGER_THRESHOLD_LIMIT = 30; // amp TODO: update limit
  private static final double TRIGGER_THRESHOLD_TIME = 0.2; // s 
  private static final double NEUTRAL_DEADBAND = 0.001;

  private static final double TICKS_PER_REV = 2048;
  private static final double GEAR_RATIO = 48.7; //TODO: update value, teeth/diameter
  private static final double TICKS_PER_ROTATION = TICKS_PER_REV * GEAR_RATIO;

  private static final double TICKS_PER_SECOND = TICKS_PER_REV;
  private static final double TICKS_PER_100_MS = TICKS_PER_REV/ 10.0;
  private static final double ONE_HUNDRED_MS_PER_MINUTE = 1000.0/600000.0;

  private static final double MIN_ROTATION_TICKS = degreesToTicks(-5); //-120
  private static final double MAX_ROTATION_TICKS = degreesToTicks(120); //0 //TODO: update number  

  private static final double MIN_ROTATION_DEGREES = ticksToDegrees(MIN_ROTATION_TICKS);
  private static final double MAX_ROTATION_DEGREES = ticksToDegrees(MAX_ROTATION_TICKS);
  private static final double TOTAL_DEGREES = Math.abs(MAX_ROTATION_DEGREES) + Math.abs(MIN_ROTATION_DEGREES);

  private static final double kPeakOutput = 1.0;
  private static final int kPIDLoopIdx = 0;
  private static final int kTimeoutMs = 1000;
  private static boolean kSensorPhase = false;
  private static TalonFXInvertType kMotorInvert = TalonFXInvertType.CounterClockwise;

  private static final double ACCELERATION_LIMIT = 12500; // should start as a little bit more than the accel //TODO: update value after testing
  private static final double CRUISE_VELOCITY_LIMIT = 25000; // should be a bit less than max possible velocity //TODO: update value after testing
  private static final double ALLOWABLE_ERROR = 15.0;
  public static Arm getInstance() {
    if (instance == null)
      instance = new Arm();
    return instance;  
  }    

  // Gains
  private static final double kP;
  private static final double kI;
  private static final double kD;
  private static final double kF;
  private static final double kArbitraryFeedForward;

  static{
    kP = 0.4;
    kI = 0.0;
    kD = 0.7; // when P is oscillating, lower P and make D gain 1.6 times P.
    kF = 0.0; // 1023 / (((estimated speed_rpm based on the fact our gear ratio won't allow our motor to go at max velocity / 60) / 10) * 2048)
    kArbitraryFeedForward = 0.035;
  }

  private static final double kV = 0.05;
  private static final double kA = 0.002;

  private Arm() {
    armMotor = new TalonFXHelper(Constants.CANDevices.ARM_MOTOR);

    //arm config
    armMotor.configFactoryDefault();
    armMotor.configNeutralDeadband(NEUTRAL_DEADBAND);
    armMotor.setNeutralMode(NeutralMode.Brake);

    armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
    armMotor.setSensorPhase(kSensorPhase);
    armMotor.setInverted(kMotorInvert);

    armMotor.configNominalOutputForward(0, kTimeoutMs);
    armMotor.configNominalOutputReverse(0, kTimeoutMs);
    armMotor.configPeakOutputForward(kPeakOutput, kTimeoutMs);
    armMotor.configPeakOutputReverse(-kPeakOutput, kTimeoutMs);

    armMotor.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
    armMotor.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
    armMotor.config_kD(kPIDLoopIdx, kD, kTimeoutMs);
    armMotor.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
    armMotor.configMotionAcceleration(ACCELERATION_LIMIT);
    armMotor.configMotionCruiseVelocity(CRUISE_VELOCITY_LIMIT);
    armMotor.configAllowableClosedloopError(0, ALLOWABLE_ERROR, kTimeoutMs);

    armMotor.configForwardSoftLimitEnable(true);
    armMotor.configReverseSoftLimitEnable(true);
    armMotor.configForwardSoftLimitThreshold(MAX_ROTATION_TICKS);
    armMotor.configReverseSoftLimitThreshold(MIN_ROTATION_TICKS);
    
    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CONTINUOUS_CURRENT_LIMIT, 
      TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);
    
    armMotor.configSupplyCurrentLimit(talonCurrentLimit);

    armMotor.configClosedLoopStatusFrameRates();

    armSim = new SingleJointedArmSim(LinearSystemId.identifyPositionSystem(kV, kA), 
                                    DCMotor.getFalcon500(1), 
                                    GEAR_RATIO, 
                                    Constants.RobotMetrics.ARM_LENGTH, 
                                    Math.toRadians(MIN_ROTATION_DEGREES), 
                                    Math.toRadians(MAX_ROTATION_DEGREES), 
                                    true);
    armMotorSim = armMotor.getSimCollection();
  }

  public static double ticksToDegrees(double ticks) {
    return (ticks/TICKS_PER_ROTATION)*360;
  }

  public static double degreesToTicks(double degrees) {
    return (degrees/360)*TICKS_PER_ROTATION;
  }

  public static double ticksPer100msToDegreesPerSecond(double ticks) {
    return ticksToDegrees(ticks) * 10.0;
  }

  public static double degreesPerSecondToTicksPer100ms(double degrees) {
    return degreesToTicks(degrees) / 10.0;
  }

  /**
   * Calculates how far out the robot arm is in degrees, <b>is always positive</b> (as if 0 is the top of the arm's range)
   * @param inches how far out the arm is in inches
   * @return how far out the arm is in degrees
   */
  public static double calculateDegreesfromInches(double inches) {
    return Math.toDegrees(Math.asin(inches/Constants.RobotMetrics.ARM_LENGTH));
  }

  /**
   * Moves the arm to a certain position
   * 0 degrees moves the arm downward, -180 moves the arm upward
   * @param degrees the desired destination (degrees)
   */
  public void setRotationDegrees(double degrees) {
    double demand = MathUtil.clamp(degrees, MIN_ROTATION_DEGREES, MAX_ROTATION_DEGREES);
    setpoint = demand;
    armMotor.set(ControlMode.MotionMagic, degreesToTicks(demand), DemandType.ArbitraryFeedForward, kArbitraryFeedForward);
  }

  /**
   * The speed to set the arm to
   * @param vel the speed to set the arm to (degrees/second)
   */
  public void setVelocity(double vel) {
    armMotor.set(ControlMode.Velocity, vel, DemandType.ArbitraryFeedForward, kArbitraryFeedForward);
  }

  public void setPercentOutput(double speed) {
    armMotor.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, kArbitraryFeedForward);
  }



  /**
   * @return the current error position (degrees)
   */
  @Log(name = "Error", rowIndex = 1, columnIndex = 1)
  public double getControllerError() {
    return ticksToDegrees(armMotor.getClosedLoopError());
  }

  /**
   * @return the velocity of the arm (degrees/second)
   */
  @Log(name = "Speed (deg/sec)", rowIndex = 2, columnIndex = 2)
  public double getVelocity() {
    return ticksPer100msToDegreesPerSecond(armMotor.getSelectedSensorVelocity());
  }

  /**
   * @return the position of the arm (degrees)
   */
  @Log(name = "Position", rowIndex = 2, columnIndex = 1)
  public double getPositionDegrees() {
    return ticksToDegrees(armMotor.getSelectedSensorPosition());
  }

  /**
   * @return the position of the arm (ticks)
   */
  @Log(name = "Encoder Position", rowIndex = 1, columnIndex = 3)
  public double getEncoderPosition() {
    return armMotor.getSelectedSensorPosition();
  }

  public boolean atSoftLimit() {
    return !(getPositionDegrees() > MIN_ROTATION_DEGREES && getPositionDegrees() < MAX_ROTATION_DEGREES);
  }

  public boolean isAtZero() {
    return (getPositionDegrees() > -0.5 && getPositionDegrees() < 0.5);
  }

  public double getForwardSoftLimit() {
    return MIN_ROTATION_DEGREES;
  }

  public double getReverseSoftLimit() {
    return MAX_ROTATION_DEGREES;
  }

  public double getSetpoint() {
    return setpoint;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic(){
    armMotorSim.setBusVoltage(RobotController.getBatteryVoltage());

    armSim.setInput(armMotorSim.getMotorOutputLeadVoltage());
    armSim.update(0.02);

    double simVelocityTicksPer100ms = degreesToTicks(Math.toDegrees(armSim.getVelocityRadPerSec()/10));
    armMotorSim.setIntegratedSensorVelocity((int) simVelocityTicksPer100ms);
    armMotorSim.setIntegratedSensorRawPosition((int) (getEncoderPosition() + 0.02*simVelocityTicksPer100ms));
  }
}
