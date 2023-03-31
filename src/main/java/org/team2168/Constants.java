// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

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
    public static final int TURRET_MOTOR = 10;
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

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }


  // public static final double LOOP_TIMESTEP_S = 0.02;
  public static final class FieldMetrics {
    public static final double MARKER_SIZE_MM = 152.4;
    public static final double apriltagHeightFromGroundMeters = Units.feetToMeters(1.1875);
    public static final double gamePieceStationApritagHeightFromGroundMeters = Units.feetToMeters(1.9479);
    public static final double LOW_NODE_LENGTH_METERS = Units.inchesToMeters(5.6875);
    public static final double MIDDLE_NODE_LENGTH_METERS = Units.inchesToMeters(22.75);
    public static final double HIGH_NODE_LENGTH_METERS = Units.inchesToMeters(39.75);
    public static final Field2d field = new Field2d();
  }
  
public static final double LOOP_TIMESTEP_S = 0.02;

  public static final class Joysticks {
    public static final int DRIVER_JOYSTICK = 0; //these constants are placeholders. 
    public static final int OPERATOR_JOYSTICK = 1;//these constants are placeholders. 
    public static final int BUTTON_BOX_1 = 4;//these constants are placeholders. 
    public static final int BUTTON_BOX_2 = 8;//these constants are placeholders. 
    public static final int DRIVER_OPERATOR_E_BACKUP = 2;//these constants are placeholders. 
    public static final int PID_TEST_JOYSTICK = 5;//these constants are placeholders. 
}

public static final class AprilTagPoses {
  //inches to meters
  public static List<Pose3d> apriltagPoses = List.of(
    new Pose3d(new Translation3d(15.513, 1.071, 0.462), new Rotation3d(0.0,0.0, 180.0)),
    new Pose3d(new Translation3d(15.513, 2.748, 0.462), new Rotation3d(0.0,0.0,180.0)),
    new Pose3d(new Translation3d(15.513, 4.424, 0.462), new Rotation3d(0.0,0.0,180.0)),
    new Pose3d(new Translation3d(16.178, 6.749, 0.695), new Rotation3d(0.0, 0.0, 180.0)),
    new Pose3d(new Translation3d(0.361, 6.749, 0.695), new Rotation3d(0.0, 0.0, 0.0)),
    new Pose3d(new Translation3d(1.027, 4.424, 0.462), new Rotation3d(0.0, 0.0, 0.0)),
    new Pose3d(new Translation3d(1.027, 2.748, 0.462), new Rotation3d(0.0, 0.0, 0.0)),
    new Pose3d(new Translation3d(1.027, 1.071, 0.462), new Rotation3d(0.0, 0.0, 0.0))
  );
}


public static final class ROBOT_METRICS {
  public static final double ARM_LENGTH_TO_CLAW = Units.inchesToMeters(55.92);
}
}



