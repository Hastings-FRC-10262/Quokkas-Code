// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveTrain drive = new DriveTrain();
  private final Arm arm = new Arm();
  private final Intake intake = new Intake();
  
  private final CommandXboxController driverController =
      new CommandXboxController(0);

  private final CommandXboxController mechanismController =
      new CommandXboxController(1);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  
  public RobotContainer() {
    configureBindings();
    configureAutos();
  }

  public void configureAutos() {
    autoChooser.setDefaultOption("Do Anything", Autos.driveForward(drive, arm, intake));
    //autoChooser.addOption("", Autos.);
    //autoChooser.addOption("", Autos.);
    //autoChooser.addOption("", Autos.);

    SmartDashboard.putData(autoChooser);
  }

  private void configureBindings() {

    // *** Drive bindings ***
    drive.setDefaultCommand(drive.driveArcade(driverController::getLeftY, driverController::getRightX));    
 
    
    // *** Arm bindings ***
    arm.setDefaultCommand(arm.moveArm(0.0));
    
    mechanismController.leftBumper()
      .and(mechanismController.leftTrigger().negate())
      .whileTrue(arm.moveArm(OperatorConstants.armSpeed));

    mechanismController.leftTrigger()
      .and(mechanismController.leftBumper().negate()) 
      .whileTrue(arm.moveArm(-1 * OperatorConstants.armSpeed));

    //PID bindings
    mechanismController.a()
      .onTrue(arm.moveArmToPosition(ArmConstants.testPosition60));
      
    mechanismController.b()
      .onTrue(arm.moveArmToPosition(ArmConstants.testPosition40));

    // Move to intake coral with Y
    mechanismController.y()
      .onTrue(arm.moveArmToPosition(ArmConstants.positionFloorIntake));

    // // Move to intake algae with X
    // mechanismController.x()
    //   .onTrue(arm.moveArmToPosition(ArmConstants.positionIntakeAlgae));

    // // Move to remove low-reef algae and dump L1 coral with B
    // mechanismController.b()
    //   .onTrue(arm.moveArmToPosition(ArmConstants.positionRemoveAlgaeLow));

    // // Move to remove high-reef algae with A
    // mechanismController.a()
    //   .onTrue(arm.moveArmToPosition(ArmConstants.positionRemoveAlgaeHigh));

    // *** Intake bindings ***
    // Default behaviour (do nothing)
    intake.setDefaultCommand(intake.moveIntake(0.0));

    // Run intake with right bumper button
    mechanismController.rightBumper()
      .and(mechanismController.rightTrigger().negate())
      .whileTrue(intake.moveIntake(0.5));

    // Run intake in reverse with right trigger button
    mechanismController.rightTrigger()
      .and(mechanismController.rightBumper().negate()) 
      .whileTrue(intake.moveIntake(-0.5));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Autos.driveForward(drive, arm, intake);
  }
}
