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
    public static Double driveForwardGain = 0.55;
    public static Double driveTurnGain = 0.4;

    public static Double armSpeed = 0.4;
  }

  public static class ArmConstants {    
     
      //Robot starting angle
      public static Double startingAngle = 54.644;

      public static Double armTolerance = 2.0;
      
      //Coral PID setpoints
      public static Double positionFloorIntake            = 1.0;
      public static Double positionDepositL1              = 33.543; //PLACEHOLDER!
      public static Double positionHumanPlayerIntake      = 59.478; //PLACEHOLDER!
      
      //public static Double testPosition40 = 40.0;
      //public static Double testPosition60 = 60.0;

      // Define Arm position limits
      public static Double armFrontLimit            = 0.0;
      public static Double armRearLimit            = 140.0;

      // Define Arm velocity limit
      public static Double armVelocityLimit         = 0.8;

      // Define Arm PID constants
      public static Double armkP                    = 0.03;
      public static Double armkI                    = 0.0;
      public static Double armkD                    = 0.0;
  }
}
