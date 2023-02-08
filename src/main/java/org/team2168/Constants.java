// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public final class CANDevices {
    public static final int INTAKE_RIGHT_MOTOR = 8;
    public static final int INTAKE_LEFT_MOTOR = 9;

  }

  public static final class PneumaticsModules {
    public static final int INTAKE_CLAMP = 1;
    public static final int INTAKE_OPEN = 2;
  
    public static final PneumaticsModuleType MODULE_TYPE = PneumaticsModuleType.REVPH;
  }

  public static final class MotorSpeeds {
    public static final double FORWARD_INTAKE_VELOCITY = 100.0; //placeholder in rpm to ticks per hundred ms
    public static final double STOP_SPEED = 0.0;
    public static final double REVERSE_INTAKE_VELOCITY = -100.0; //placeholder in rpm to ticks per hundred ms
  }

  public static final class DIO {
  public static final int HAND_CHANNEL = 1;
  }

  public static final class FieldMetrics {
    public static final double MARKER_SIZE_MM = 152.4;
    public static final double apriltagHeightFromGroundMeters = Units.feetToMeters(1.1875);
    public static final double gamePieceStationApritagHeightFromGroundMeters = Units.feetToMeters(1.9479);
  }
}
