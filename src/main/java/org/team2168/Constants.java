// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class VisionConstants {
    public static final String CAMERA_NAME = "Arducam_OV9281_USB_Camera";
    public static final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), // change for real distance between photonvision cam and center of robot.
    new Rotation3d(0, 0, 0)
    );
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
