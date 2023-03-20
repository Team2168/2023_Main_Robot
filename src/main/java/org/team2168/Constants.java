// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class CANDevices {
    public static final int DRIVETRAIN_LEFT_MOTOR_1 = 18; // uses placeholder value
    public static final int DRIVETRAIN_LEFT_MOTOR_2 = 19; // uses placeholder value
    public static final int DRIVETRAIN_RIGHT_MOTOR_1 = 1; // uses placeholder value
    public static final int DRIVETRAIN_RIGHT_MOTOR_2 = 0; // uses placeholder value
    public static final int PIGEON_IMU = 20; // uses placeholder value
  }

  public static class Drivetrain {
    public final static int kPigeonUnitsPerRotation = 8192;
    public static final double MAX_VOLTAGE = 10.0;
    public static final double ksVolts = 0.16855;
    public static final double kvVoltSecondsPerMeter = 2.3191;
    public static final double kaVoltSecondsSquaredPerMeter = 0.28327;

    public static final double kTrackwidthMeters = 0.5842; // uses physical trackwidth, need to average with sysid results
    public static final DifferentialDriveKinematics kDriveKinematics = 
      new DifferentialDriveKinematics(kTrackwidthMeters);

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2.0; // replace these for 2022
    public static final double kRamseteZeta = 0.7;

    public static final double kPDriveVel = 2.9277;

    public static final double kMaxSpeedMetersPerSecond = 3.0;  // taken from sysID
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.8;

  }


  public static final class ElevatorMotors{
    public static final int ELEVATOR_MOTOR = 14; 
    public static final double UPDATE_TIME = 0.02;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class FieldMetrics {
    public static final double MARKER_SIZE_MM = 152.4;
    public static final double apriltagHeightFromGroundMeters = Units.feetToMeters(1.1875);
    public static final double gamePieceStationApritagHeightFromGroundMeters = Units.feetToMeters(1.9479);
    public static final double MIDDLE_CUBE_NODE_HEIGHT_IN = 23.5;
    public static final double TOP_CUBE_NODE_HEIGHT_IN = 35.5;
    public static final double MIDDLE_CONE_NODE_HEIGHT_IN = 34;
    public static final double TOP_CONE_NODE_HEIGHT_IN = 46;
  }

  public static final class Joysticks {
    public static final int DRIVER_JOYSTICK = 0; //these constants are placeholders. 
    public static final int OPERATOR_JOYSTICK = 1;//these constants are placeholders. 
    public static final int BUTTON_BOX_1 = 4;//these constants are placeholders. 
    public static final int BUTTON_BOX_2 = 8;//these constants are placeholders. 
    public static final int DRIVER_OPERATOR_E_BACKUP = 2;//these constants are placeholders. 
    public static final int PID_TEST_JOYSTICK = 5;//these constants are placeholders. 
}
}

