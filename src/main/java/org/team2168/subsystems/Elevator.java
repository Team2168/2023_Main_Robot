// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;

import java.security.KeyFactory;

import org.team2168.Constants;
import org.team2168.Constants.ElevatorMotors;
import org.team2168.Constants.PneumaticDevices;
import org.team2168.utils.TalonFXHelper;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Elevator extends SubsystemBase implements Loggable {

  private static final double kI = 0.0; //intergral (placeholder) (used for super specific scenarios)
  private static final double kD = 0.8; //derivative (placeholder) (if it is oscilating too much you should add a small d gain but otherwise you should just use a p gain) (0.8 works - ted)
  private static final double kF = 0.12; // 0.12 works - ted
  // private static final double kF = ((1023) / (0.75 * 21777.07)); //feedforward: constant output added on which counteracts forces (0.0626)
  private static final double kP = 0.5; //proportional: a proportion of the input (placeholder) (increase this until it reaches oscilation and then decrease this once it reaches that point) (0.5 works - ted)
  private static final double kArbitraryFeedForward = 0.12; //(placeholder)

  private static final int kTimeoutMs = 30; //how long it takes for the config to configure in Ms
  private static final int kPIDLoopIdx = 0; //constant for id purposes

  private static final double CURRENT_LIMIT = 40.0; //it limits when the feature is activited (in amps)
  private static final double THRESHOLD_CURRENT = 0.0; //it tells what the threshold should be for the limit to be activited (in amps)
  private static final double THRESHOLD_TIME = 0.2; //time in seconds of when the limiting should happen after the threshold has been overreached

  private static final double TIME_UNITS_OF_VELOCITY = 0.1; //in seconds 
  private static final double TICKS_PER_REV = 2048;
  private static final double GEAR_RATIO = 4.5; //((8/48) * (28/42) * (40/20)); 
  private static final double SPROCKET_RADIUS = 0.625; 
  private static final double INCHES_PER_REV = SPROCKET_RADIUS * 2 * Math.PI;


  private static final double kPeakOutput = 1.0;
  private static final double NEUTRAL_DEADBAND = 0.001; 
  private static final double ACCELERATION_LIMIT = inchesToTicks(0.8) * TIME_UNITS_OF_VELOCITY; //(TODO:placeholder)
  private static final double CRUISE_VELOCITY_LIMIT = inchesToTicks(0.45) * TIME_UNITS_OF_VELOCITY; //(TODO: placeholder)

  private static TalonFXInvertType kInvertType = TalonFXInvertType.Clockwise; //this inverts the rotation of the motors so that the shaft goes up (clockwise)

  private TalonFXHelper elevatorMotor;
  
  //private double position; //height in inches
  private SupplyCurrentLimitConfiguration talonCurrentLimit = new SupplyCurrentLimitConfiguration(true, CURRENT_LIMIT, THRESHOLD_CURRENT, THRESHOLD_TIME);
  private static ElevatorSim elevatorSim;
  private static TalonFXSimCollection elevatorMotorSim;
  private static final double CARRIAGE_MASS_KG = 4.5; //(placeholder)
  private static final double MIN_HEIGHT_INCHES = -25.0; //+11.9 (30.1 inches is the distance from top of frame to top of moving piece)
  private static final double MAX_HEIGHT_INCHES = 0; 

  private DoubleSolenoid carriageLock;

  private boolean kSensorPhase = false;

  static Elevator instance = null;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor = new TalonFXHelper(ElevatorMotors.ELEVATOR_MOTOR); //these are placeholder constant values
    carriageLock = new DoubleSolenoid(PneumaticDevices.MODULE_TYPE, PneumaticDevices.CARRIAGE_LOCK_OPEN, PneumaticDevices.CARRIAGE_LOCK_CLOSE);

    elevatorMotor.configNeutralDeadband(NEUTRAL_DEADBAND);    
    elevatorMotor.setNeutralMode(NeutralMode.Brake);

    elevatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
    elevatorMotor.setSensorPhase(kSensorPhase);

    elevatorMotor.configNominalOutputForward(0, kTimeoutMs);
    elevatorMotor.configNominalOutputForward(0, kTimeoutMs);
    elevatorMotor.configPeakOutputForward(kPeakOutput, kTimeoutMs);
    elevatorMotor.configPeakOutputReverse(kPeakOutput, kTimeoutMs);

    //sets the gains
    elevatorMotor.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
    elevatorMotor.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
    elevatorMotor.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
    elevatorMotor.config_kD(kPIDLoopIdx, kD, kTimeoutMs);
    elevatorMotor.configMotionAcceleration(ACCELERATION_LIMIT);
    elevatorMotor.configMotionCruiseVelocity(CRUISE_VELOCITY_LIMIT);
    elevatorMotor.configAllowableClosedloopError(0, 15, kTimeoutMs); // 15 works - ted

    elevatorMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen); //this is subject to change
    elevatorMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    // elevatorMotor.configForwardSoftLimitEnable(true);
    // elevatorMotor.configReverseSoftLimitEnable(true);
    // elevatorMotor.configForwardSoftLimitThreshold(inchesToTicks(MAX_HEIGHT_INCHES));
    // elevatorMotor.configReverseSoftLimitThreshold(inchesToTicks(MIN_HEIGHT_INCHES));
    talonCurrentLimit = new SupplyCurrentLimitConfiguration(true, CURRENT_LIMIT, THRESHOLD_CURRENT, THRESHOLD_TIME);

    //puts limis on the input (configs)
    elevatorMotor.configSupplyCurrentLimit(talonCurrentLimit);

    elevatorMotor.setInverted(kInvertType);
    elevatorMotor.configClosedLoopStatusFrameRates();

    elevatorMotor.configFactoryDefault();
   
    elevatorSim = new ElevatorSim(
    DCMotor.getFalcon500(1), 
    GEAR_RATIO, 
    CARRIAGE_MASS_KG, 
    Units.inchesToMeters(SPROCKET_RADIUS), 
    Units.inchesToMeters(MIN_HEIGHT_INCHES), 
    Units.inchesToMeters(MAX_HEIGHT_INCHES), 
    kSensorPhase, 
    VecBuilder.fill(0.1));

    elevatorMotorSim = elevatorMotor.getSimCollection();

    //public EncoderSim encoderSim = new EncoderSim(encoder);
  }

  public static Elevator getInstance(){
    if (instance == null){
      instance = new Elevator();
    }
    return instance;
  }


  public static double degreesToTicks(double degrees){
    return (degrees / 360 * TICKS_PER_REV);
  }

  public static double ticksToDegrees(double ticks){
    return (ticks / TICKS_PER_REV * 360);
  }

  public static double inchesToTicks(double inches){
    return(inches/INCHES_PER_REV) * GEAR_RATIO * TICKS_PER_REV;
  }

  public boolean isInRange() {
    return (getPositionIn() > MIN_HEIGHT_INCHES && getPositionIn() <= MAX_HEIGHT_INCHES);
  }

  public static double ticksToInches(double ticks){
    return ((ticks / TICKS_PER_REV) /GEAR_RATIO) * INCHES_PER_REV;
  }

  @Log(name = "Encoder Position in Ticks")
  private double getEncoderPositionInTicks(){
    return elevatorMotor.getSelectedSensorPosition(kPIDLoopIdx);
  }

  public void setEncoderPosZero(){
    elevatorMotor.setSelectedSensorPosition(0);
  }

  public void setMotorBrake() {
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setMotorCoast() {
    elevatorMotor.setNeutralMode(NeutralMode.Coast);
  }

  //Config()
  public void setSpeedVelocity(double speed) {
    elevatorMotor.set(ControlMode.Velocity, inchesToTicks(speed) * TIME_UNITS_OF_VELOCITY); //the "speed" parameter is the rate of the movement per second (in inches)
  }

  //@Config()
  public void setPosition(double inches){
    //this.position = position;

    elevatorMotor.set(ControlMode.MotionMagic, inchesToTicks(inches));
  }

  //@Config()
  public void setPercentOutput(double percentOutput) {
    elevatorMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setToZero(){
    elevatorMotor.set(ControlMode.PercentOutput, 0);
  }

  public void extendLock(){
    carriageLock.set(DoubleSolenoid.Value.kForward);
  }

  public void retractLock(){
    carriageLock.set(DoubleSolenoid.Value.kReverse);
  }

  @Log(name = "Positiion (inches)", rowIndex = 3, columnIndex = 2)
  public double getPositionIn(){
    return ticksToInches(elevatorMotor.getSelectedSensorPosition());
  }

  @Log(name = "Speed", rowIndex = 3, columnIndex = 3)
  public double getSpeed(){
    return elevatorMotor.get();
  }

  @Log(name = "Velocity (inches / sec)", rowIndex = 3, columnIndex = 4)
  public double getVelocity(){
    return (ticksToInches(elevatorMotor.getSelectedSensorVelocity())) / TIME_UNITS_OF_VELOCITY;
  }

  @Log(name = "At Zero", rowIndex = 3, columnIndex = 0)
  public boolean isZeroPosition(){
    return elevatorMotor.isRevLimitSwitchClosed() == 1;
  }

  @Log(name = "At Top", rowIndex = 3, columnIndex = 1)
  public boolean isAtUpperPosition(){
    return elevatorMotor.isFwdLimitSwitchClosed() == 1;
  }

  @Log(name = "Error", rowIndex = 3, columnIndex = 5)
  public double getControllerError(){
    return ticksToDegrees(elevatorMotor.getClosedLoopError()); //this method returns the current error position
  }

  @Log(name = "is Lock extended?")
  public boolean isLockExtended(){
    return !carriageLock.isFwdSolenoidDisabled();
  }

  @Log(name = "is Lock retracted?")
  public boolean isLockRetracted(){
    return !carriageLock.isRevSolenoidDisabled();
  }



  @Override
  public void periodic() {
}

@Override
public void simulationPeriodic() {
  // This method will be called once per scheduler run
  elevatorMotorSim.setBusVoltage(getSpeed() * RobotController.getBatteryVoltage()); //sets output of motor with speed and voltage
  elevatorSim.setInput(elevatorMotorSim.getMotorOutputLeadVoltage()); //gets motor output
  elevatorSim.update(Constants.ElevatorMotors.UPDATE_TIME); //how often the elevator will update (in secs)

  double simVelocityInTicks = inchesToTicks(getSpeed() * TIME_UNITS_OF_VELOCITY);
  double simPosition = inchesToTicks(getPositionIn());

  elevatorMotorSim.setIntegratedSensorRawPosition((int) simPosition);
  elevatorMotorSim.setIntegratedSensorVelocity((int) simVelocityInTicks);
}
}
