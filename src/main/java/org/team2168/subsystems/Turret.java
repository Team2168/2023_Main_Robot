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
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;

import org.team2168.Constants;
import org.team2168.utils.Gains;
import org.team2168.utils.TalonFXHelper;



public class Turret extends SubsystemBase implements Loggable {

  private static TalonFXHelper turretMotor;
  private static Turret instance = null;

  private static final double TICKS_PER_REV = 2048;
  private static final double GEAR_RATIO = 8.0/1.0 * 30.0/1.0; // Gear ratio maybe?
  private static final double TICKS_PER_TURRET_ROTATION = TICKS_PER_REV * GEAR_RATIO;
  private static double setpoint = 0.0;

  private static final double TICKS_PER_SECOND = TICKS_PER_TURRET_ROTATION;
  private static final double TICKS_PER_100_MS = TICKS_PER_SECOND / 10.0;
  private static final double ONE_HUNDRED_MS_PER_MINUTE = 100.0 / 60000.0;

  // 2022 values

  // The Minimum and Maximum rotation ticks of the turret are 90 degrees in both directions
  private static final int MIN_ROTATION_TICKS = -122800; 
  private static final int MAX_ROTATION_TICKS = 122800; 

  private static final double MIN_ROTATION_DEGREES = ticksToDegrees(-122800.0);
  private static final double MAX_ROTATION_DEGREES = ticksToDegrees(122800.0);
  private static final double TOTAL_ROTATION_DEGREES = Math.abs(MIN_ROTATION_DEGREES) + Math.abs(MAX_ROTATION_DEGREES);

  private static final double ACCELERATION = degreesPerSecondToTicksPer100ms(360.0 * 22.5);
  private static final double CRUISE_VELOCITY = degreesPerSecondToTicksPer100ms(360.0 * 10);


  public static final double kV = 0.005;
  public static final double kA = 0.0005;

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
  
  /**
   * The Gains for the Turret
   */
  static {
      kGains = new Gains(0.01, 0.0, 0.01, 0.0, 550, 1.0);
   }



  private static FlywheelSim turretSim;
  private static TalonFXSimCollection turretMotorSim;

    /** Creates a new Turret. */
  private Turret() {
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

  public static double getForwardSoftLimit() {
    return ticksToDegrees(MAX_ROTATION_TICKS);
  }

  public static double getReverseSoftLimit() {
    return ticksToDegrees(MIN_ROTATION_TICKS);
  }

  /**
   * 
   * @param degrees
   * Sets the rotation speed of the Turret using MotionMagic control mode
   */

  public void setRotationDegrees(double degrees) {
    var demand = MathUtil.clamp(degrees, ticksToDegrees(MIN_ROTATION_TICKS), ticksToDegrees(MAX_ROTATION_TICKS));
    setpoint = degrees;
    turretMotor.set(ControlMode.MotionMagic, degreesToEncoderTicks(demand));
  }


  /**
   * 
   * @param degrees
   * Sets the rotation speed of the Turret using the Velocity control mode
   */
  public void setVelocity(double degrees) {
    turretMotor.set(ControlMode.Velocity, degreesPerSecondToTicksPer100ms(degrees));
  }

  /**
   * 
   * @param speed
   * Sets the rotation speed of the Turret using the PercentOutput control mode
   */
  public void setSpeed(double speed) {
    turretMotor.set(ControlMode.PercentOutput, speed);
  }


  /**
   * 
   * @return Encoder Position
   * Finds the position of the encoder on the Falcon500
   */
  public double getEncoderPosition() {
    return turretMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * 
   * @return Turret Angle
   * Gets the angle the turret is facing using the Encoder position and some more extra calculations
   */
  @Log(name = "TurretPosition (deg)", rowIndex = 1, columnIndex = 1)
  public double getTurretAngle() {
    return (turretMotor.getSelectedSensorPosition() / TICKS_PER_REV * 360 / GEAR_RATIO);
  }

  @Override
  public void simulationPeriodic() {
    // Affect motor outputs by main system battery voltage dip 
    turretMotorSim.setBusVoltage(RobotController.getBatteryVoltage());

    // Pass motor output voltage to physics sim
    turretSim.setInput(turretMotorSim.getMotorOutputLeadVoltage());
    turretSim.update(Constants.LOOP_TIMESTEP_S);

    // Update motor sensor states based on physics model
    double sim_velocity_ticks_per_100ms = turretSim.getAngularVelocityRPM() / ONE_HUNDRED_MS_PER_MINUTE;
    turretMotorSim.setIntegratedSensorVelocity((int) sim_velocity_ticks_per_100ms);
    turretMotorSim.setIntegratedSensorRawPosition((int) (getEncoderPosition() + (Constants.LOOP_TIMESTEP_S / 10) * sim_velocity_ticks_per_100ms));

  }
}
