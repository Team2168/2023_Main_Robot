// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;

import org.team2168.Constants;
import org.team2168.utils.Gains;
import org.team2168.utils.TalonFXHelper;



public class Turret extends SubsystemBase {

  private static TalonFXHelper turretMotor;
  private static Turret instance = null;

  private static final double TICKS_PER_REV = 2048;
  private static final double GEAR_RATIO = 8.0/1.0 * 30.0/1.0; // Gear ratio maybe?
  private static final double TICKS_PER_TURRET_ROTATION = TICKS_PER_REV * GEAR_RATIO;
  private static double setpoint = 0.0;

  private static final double TICKS_PER_SECOND = TICKS_PER_TURRET_ROTATION;
  private static final double TICKS_PER_100_MS = TICKS_PER_SECOND / 10.0;
  private static final double ONE_HUNDRED_MS_PER_MINUTE = 100.0 / 60.0;

  // 2022 values, TODO: Update to new robots requirements.
  private static final int MIN_ROTATION_TICKS = -73400; 
  private static final int MAX_ROTATION_TICKS = 52200; 

  private static final double MIN_ROTATION_DEGREES = ticksToDegrees(-73400.0);
  private static final double MAX_ROTATION_DEGREES = ticksToDegrees(52200.0);
  private static final double TOTAL_ROTATION_DEGREES = Math.abs(MIN_ROTATION_DEGREES) + Math.abs(MAX_ROTATION_DEGREES);

  private static final double ACCELERATION = degreesPerSecondToTicksPer100ms(360.0 * 4.5);
  private static final double CRUISE_VELOCITY = degreesPerSecondToTicksPer100ms(360.0 * 2.0);

  public static final double kV = 0.02;
  public static final double kA = 0.002;

  public static final int kPIDLoopIdx = 0;
  public static final int kTimeoutMs = 30;
  public static boolean kSensorPhase = true;
  public static boolean kMotorInvert = false;

  private SupplyCurrentLimitConfiguration talonCurrentLimit; // PLACEHOLDERS
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 30; //amps
  private final double TRIGGER_THRESHOLD_LIMIT = 40; //amps
  private final double TRIGGER_THRESHOLD_TIME = 0.02; //seconds

  private static final Gains kGains; 
  
  static {
      kGains = new Gains(0.014, 0.0, 0.0, 0.0, 550, 1.0);
   }



  private static FlywheelSim turretSim;
  private static TalonFXSimCollection turretMotorSim;

    /** Creates a new Turret. */
  public Turret() {
    turretMotor = new TalonFXHelper(Constants.CANDevices.TURRET_MOTOR);

    talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
    CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);

    turretMotor.configFactoryDefault();

    turretMotor.configClosedLoopStatusFrameRates();

    turretMotor.configSupplyCurrentLimit(talonCurrentLimit);

    turretMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
    turretMotor.setSensorPhase(kSensorPhase);
    turretMotor.setInverted(kMotorInvert);
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.configNeutralDeadband(0.001);

    turretMotor.configForwardSoftLimitThreshold(degreesToEncoderTicks(MAX_ROTATION_DEGREES));
    turretMotor.configReverseSoftLimitThreshold(degreesToEncoderTicks(MIN_ROTATION_DEGREES));
    turretMotor.configForwardSoftLimitEnable(true);
    turretMotor.configReverseSoftLimitEnable(true);


    turretMotor.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);

    turretMotor.config_kF(kPIDLoopIdx, kGains.kF, kTimeoutMs);
    turretMotor.config_kP(kPIDLoopIdx, kGains.kP, kTimeoutMs);
    turretMotor.config_kI(kPIDLoopIdx, kGains.kI, kTimeoutMs);
    turretMotor.config_kD(kPIDLoopIdx, kGains.kD, kTimeoutMs);

    turretMotor.configMotionAcceleration(ACCELERATION);
    turretMotor.configMotionCruiseVelocity(CRUISE_VELOCITY);
  
    turretSim = new FlywheelSim
    (LinearSystemId.identifyVelocitySystem(kV, kA), // Add silly lil gains at somepoint please
    DCMotor.getFalcon500(1), 
    GEAR_RATIO);

    turretMotorSim = turretMotor.getSimCollection();
 }

 public static Turret getInstance() {
  if (instance == null)
    instance = new Turret();
    return instance;
 }

  private static double ticksToDegrees(double ticks) {
    return (ticks / TICKS_PER_TURRET_ROTATION) * 360.0;
  }

  private static double degreesToEncoderTicks(double degrees) {
    return (degrees/360.0) * TICKS_PER_TURRET_ROTATION;
  }

  private static double ticksPer100msToDegreesPerSecond(double ticks) {
    return ticksToDegrees(ticks) * 10.0;
  }

  public static double degreesPerSecondToTicksPer100ms(double degrees) {
    return degreesToEncoderTicks(degrees) / 10.0;
  }

  public void setRotationDegrees(double degrees) {
    var demand = MathUtil.clamp(degrees, ticksToDegrees(MIN_ROTATION_TICKS), ticksToDegrees(MAX_ROTATION_TICKS));
    setpoint = degrees;
    turretMotor.set(ControlMode.MotionMagic, degreesToEncoderTicks(demand));
  }

  public void setVelocity(double degrees) {
    turretMotor.set(ControlMode.Velocity, degreesPerSecondToTicksPer100ms(degrees));
  }

  public void setSpeed(double speed) {
    turretMotor.set(ControlMode.PercentOutput, speed);
  }

  public double getEncoderPosition() {
    return turretMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // Affect motor outputs by main system battery voltage dip 
    turretMotorSim.setBusVoltage(RobotController.getBatteryVoltage());

    // Pass motor output voltage to physics sim
    turretSim.setInput(turretMotorSim.getMotorOutputLeadVoltage());
    turretSim.update(Constants.LOOP_TIMESTEP_S);

    // Update motor sensor states based on physics model
    double sim_velocity_ticks_per_100ms = turretSim.getAngularVelocityRPM() * ONE_HUNDRED_MS_PER_MINUTE;
    turretMotorSim.setIntegratedSensorVelocity((int) sim_velocity_ticks_per_100ms);
    turretMotorSim.setIntegratedSensorRawPosition((int) (getEncoderPosition() + Constants.LOOP_TIMESTEP_S * sim_velocity_ticks_per_100ms));

  }
}
