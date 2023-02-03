// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

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

  public static final class Joysticks {
    public static final int DRIVER_JOYSTICK = 6; //these constants are placeholders. 
    public static final int OPERATOR_JOYSTICK = 9;//these constants are placeholders. 
    public static final int BUTTON_BOX_1 = 4;//these constants are placeholders. 
    public static final int BUTTON_BOX_2 = 8;//these constants are placeholders. 
    public static final int DRIVER_OPERATOR_E_BACKUP = 2;//these constants are placeholders. 
    public static final int PID_TEST_JOYSTICK = 7;//these constants are placeholders. 
}

}
