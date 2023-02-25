// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
  public static class FieldConstants {
    public static final Pose3d APRIL_TAG_ID_1 = new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, 180.0));
    public static final Pose3d APRIL_TAG_ID_2 = new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, 180.0));
    public static final Pose3d APRIL_TAG_ID_3 = new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, 180.0));
    public static final Pose3d APRIL_TAG_ID_4 = new Pose3d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d(0.0, 0.0, 180.0));

    public static final Pose3d APRIL_TAG_ID_5 = new Pose3d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d(0.0, 0.0, 0.0));
    public static final Pose3d APRIL_TAG_ID_6 = new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, 0.0));
    public static final Pose3d APRIL_TAG_ID_7 = new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, 0.0));
    public static final Pose3d APRIL_TAG_ID_8 = new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, 0.0));
  }

  public static class VisionConstants {
    public static final String CAMERA_NAME = "Arducam_OV9281_USB_Camera";
    public static final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), // change for real distance between photonvision cam and center of robot.
    new Rotation3d(0, 0, 0)
    );
  }
  public static final class CANDevices {
    public static final int TURRET_MOTOR = 10; // TODO: STAND-IN VALUE FOR TESTING PURPOSES
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final double LOOP_TIMESTEP_S = 0.02;
  public static final class FieldMetrics {
    public static final double MARKER_SIZE_MM = 152.4;
    public static final double apriltagHeightFromGroundMeters = Units.feetToMeters(1.1875);
    public static final double gamePieceStationApritagHeightFromGroundMeters = Units.feetToMeters(1.9479);
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

