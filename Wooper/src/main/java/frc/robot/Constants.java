// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static class ArmConstants {
      // Define Arm position constants
      // public static Double positionIntakeCoral      = 0.422;
      // public static Double positionClimbEnd         = 0.368;
      // public static Double positionIntakeAlgae      = 0.348;
      // public static Double positionRemoveAlgaeLow   = 0.3083;
      // public static Double positionClimbStart       = 0.233;
      // public static Double positionRemoveAlgaeHigh  = 0.1;

      public static Double testPosition30 = 30.0;
      public static Double testPosition60 = 60.0;

      // Define Arm position limits
      public static Double armFrontLimit            = 20.0;
      public static Double armRearLimit             = 90.0;

      // Define Arm velocity limit
      public static Double armVelocityLimit         = 0.8;

      // Define Arm PID constants
      public static Double armkP                    = 0.0;
      public static Double armkI                    = 0.0;
      public static Double armkD                    = 0.0;
  }
}
