// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public final class Autos {

  

  public static Command driveForward(DriveTrain drive, Arm arm, Intake intake) {
    System.out.println("STARTED");
    return drive.moveStraight(0.25).withTimeout(1.4).andThen(drive.moveStraight(0.0));
  }


  /** Example static factory for an autonomous command. */
  public static Command autoSideLeft(DriveTrain drive, Arm arm, Intake intake) {


    System.out.println("LEFT");
    // Drive straight
    return drive.moveStraight(-0.25).withTimeout(2.75).

    // Turn to face reef wall
    andThen(drive.turn(-0.25).withTimeout(1.0)).

    // Move to reef wall
    andThen(drive.moveStraight(-0.20).withTimeout(1.0)).

    // Run intake while flush with reef wall, depositing L1 coral
    andThen(intake.moveIntake(-1.0).withTimeout(3.0)).

    // Move backwards with intake still running to dislodge algae
    andThen(intake.moveIntake(-1.0).withTimeout(3.0).alongWith(drive.moveStraight(0.1).withTimeout(3)));

    // Do all of the above while maintaining arm position at the 'remove low algae' position
    //alongWith(arm.moveArmToPosition(ArmConstants.positionRemoveAlgaeLow)).repeatedly();
  }
  public static Command autoSideMiddle(DriveTrain drive, Arm arm, Intake intake){
    // Drive straight

    System.out.println("MID");

    return drive.moveStraight(-0.25).withTimeout(2.75).

    

    // Move to reef wall
    andThen(drive.moveStraight(-0.20).withTimeout(1.0)).
    //need to find the angle for what the arm should be
    // Run intake while flush with reef wall, depositing L1 coral
    andThen(intake.moveIntake(-1.0).withTimeout(3.0)).
    //turn around
    andThen(drive.turn(-0.20).withTimeout(1.0)).
    
    andThen(drive.moveStraight(-0.20).withTimeout(1.0)).

    // Move backwards with intake still running to dislodge algae
    andThen(intake.moveIntake(-1.0).withTimeout(3.0).alongWith(drive.moveStraight(0.1).withTimeout(3)));

    // Do all of the above while maintaining arm position at the 'remove low algae' position
    //alongWith(arm.moveArmToPosition(ArmConstants.positionRemoveAlgaeLow)).repeatedly();
  }
  public static Command autoSideRight(DriveTrain drive, Arm arm, Intake intake){
    // Drive straight
    return drive.moveStraight(-0.25).withTimeout(2.75).

    // Move to reef wall
    andThen(drive.moveStraight(-0.20).withTimeout(1.0)).
    //need to find the angle for what the arm should be
    // Run intake while flush with reef wall, depositing L1 coral
    andThen(intake.moveIntake(-1.0).withTimeout(3.0)).
    //turn around
    andThen(drive.turn(-0.20).withTimeout(1.0)).
    
    andThen(drive.moveStraight(-0.20).withTimeout(1.0)).

    // Move backwards with intake still running to dislodge algae
    andThen(intake.moveIntake(-1.0).withTimeout(3.0).alongWith(drive.moveStraight(0.1).withTimeout(3)));

    // Do all of the above while maintaining arm position at the 'remove low algae' position
    //alongWith(arm.moveArmToPosition(ArmConstants.positionRemoveAlgaeLow)).repeatedly();
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
