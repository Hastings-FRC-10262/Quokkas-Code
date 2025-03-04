// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.*;
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
    // Default behaviour (follow Y-axes of joysticks to implement tank drive)
    
    
    drive.setDefaultCommand(drive.driveTank(driverController::getLeftY, driverController::getRightX));    
    

    // *** Arm bindings ***



    //Manually move arm slowly 
    //arm.setDefaultCommand(arm.moveArm(driverController.getLeftY()));
    driverController.b().onTrue(arm.moveArm(0.15));

    driverController.y().onTrue(arm.moveArm(-0.15));

    driverController.x().onTrue(arm.moveArm(0.0));
    
    // // Move to intake coral with Y
    // m_driverController.y()
    //   .onTrue(m_arm.moveArmToPosition(ArmConstants.positionIntakeCoral));

    // // Move to intake algae with X
    // m_driverController.x()
    //   .onTrue(m_arm.moveArmToPosition(ArmConstants.positionIntakeAlgae));

    // // Move to remove low-reef algae and dump L1 coral with B
    // m_driverController.b()
    //   .onTrue(m_arm.moveArmToPosition(ArmConstants.positionRemoveAlgaeLow));

    // // Move to remove high-reef algae with A
    // m_driverController.a()
    //   .onTrue(m_arm.moveArmToPosition(ArmConstants.positionRemoveAlgaeHigh));



    
    // *** Intake bindings ***
    // Default behaviour (do nothing)
    intake.setDefaultCommand(intake.moveIntake(0.0));

    // Run intake with right bumper button
    driverController.rightBumper()
      .and(driverController.rightTrigger().negate())
      .whileTrue(intake.moveIntake(0.25));

    // Run intake in reverse with right trigger button
    driverController.rightTrigger()
      .and(driverController.rightBumper().negate()) 
      .whileTrue(intake.moveIntake(-0.25));

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
