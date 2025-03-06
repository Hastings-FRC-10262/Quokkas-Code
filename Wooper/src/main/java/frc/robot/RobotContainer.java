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
  // The robot's subsystems and commands are defined here...
  private final DriveTrain drive = new DriveTrain();
  private final Arm arm = new Arm();
  private final Intake intake = new Intake();
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController mechanismController =
      new CommandXboxController(1);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // *** Drive bindings ***
    drive.setDefaultCommand(drive.driveArcade(driverController::getLeftY, driverController::getRightX));    
 
    //Manually move arm slowly 
    //arm.setDefaultCommand(arm.moveArm(driverController.getLeftY()));
    // mechanismController.b().onTrue(arm.moveArm(0.15));

    // mechanismController.y().onTrue(arm.moveArm(-0.15));

    // mechanismController.x().onTrue(arm.moveArm(0.0));
    

    arm.setDefaultCommand(arm.moveArm(0.0));
    
    // Run intake with right bumper button
    mechanismController.leftBumper()
      .and(mechanismController.leftTrigger().negate())
      .whileTrue(arm.moveArm(0.15));

    // Run intake in reverse with right trigger button
    mechanismController.leftTrigger()
      .and(mechanismController.leftBumper().negate()) 
      .whileTrue(arm.moveArm(-0.15));


    // Move to intake coral with Y
    mechanismController.y()
      .onTrue(arm.moveArmToPosition(ArmConstants.positionIntakeCoral));

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
      .whileTrue(intake.moveIntake(0.4));

    // Run intake in reverse with right trigger button
    mechanismController.rightTrigger()
      .and(mechanismController.rightBumper().negate()) 
      .whileTrue(intake.moveIntake(-0.4));
    
      
    
      mechanismController.a()
      .onTrue(arm.moveArmToPosition(ArmConstants.testPosition60));
      
      mechanismController.b()
      .onTrue(arm.moveArmToPosition(ArmConstants.testPosition40));
  }

  public void configureAutos() {
    autoChooser.setDefaultOption("Do Anything", Autos.driveForward(drive, arm, intake));
    //autoChooser.addOption("", Autos.);
    //autoChooser.addOption("", Autos.);
    //autoChooser.addOption("", Autos.);

    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;         //Autos.autoSideLeft(drive, arm, intake);
  }
}
