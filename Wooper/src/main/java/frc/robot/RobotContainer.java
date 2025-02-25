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

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_drive = new DriveTrain();
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake();
  private final Climber m_climber = new Climber();

  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);
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
    // *** Drive bindings ***
    // Default behavior (follow Y-axes of joysticks to implement tank drive)
    m_drive.setDefaultCommand(
        m_drive.driveTank(
            () -> -m_driverController.getRawAxis(1), // Left stick Y-axis (inverted)
            () -> -m_driverController.getRawAxis(3)  // Right stick Y-axis (inverted, may need testing)
        )
    );}

    // // *** Arm bindings ***
    // // Move to intake coral with Y (Button 4)
    // new JoystickButton(m_driverController, 4)
    //     .onTrue(m_arm.moveArmToPosition(ArmConstants.positionIntakeCoral));

    // // Move to intake algae with X (Button 1)
    // new JoystickButton(m_driverController, 1)
    //     .onTrue(m_arm.moveArmToPosition(ArmConstants.positionIntakeAlgae));

    // // Move to remove low-reef algae and dump L1 coral with B (Button 3)
    // new JoystickButton(m_driverController, 3)
    //     .onTrue(m_arm.moveArmToPosition(ArmConstants.positionRemoveAlgaeLow));

    // // Move to remove high-reef algae with A (Button 2)
    // new JoystickButton(m_driverController, 2)
    //     .onTrue(m_arm.moveArmToPosition(ArmConstants.positionRemoveAlgaeHigh));

    // // Move to start climb with D-Pad Down (POV 180°)
    // new POVButton(m_driverController, 180)
    //     .onTrue(m_arm.moveArmToPosition(ArmConstants.positionClimbStart));

    // // Move to finish climb with D-Pad Up (POV 0°)
    // new POVButton(m_driverController, 0)
    //     .onTrue(m_arm.moveArmToPosition(ArmConstants.positionClimbEnd));

    // // *** Intake bindings ***
    // // Default behavior (do nothing)
    // m_intake.setDefaultCommand(m_intake.moveIntake(0.0));

    // // Run intake with right bumper (Button 6)
    // new JoystickButton(m_driverController, 6)
    //     .whileTrue(m_intake.moveIntake(0.75));

    // // Run intake in reverse with right trigger (Axis 3)
    // new JoystickButton(m_driverController, 7) // No exact button for RT, using "Mode" button as an alternative
    //     .whileTrue(m_intake.moveIntake(-0.75));

    // // *** Climber bindings ***
    // // Default behavior (do nothing)
    // m_climber.setDefaultCommand(m_climber.moveClimber(0.0));

    // // Disengage climber with left bumper (Button 5)
    // new JoystickButton(m_driverController, 5)
    //     .whileTrue(m_climber.moveClimber(0.5));

    // // Engage climber with start button (Button 8)
    // new JoystickButton(m_driverController, 8)
    //     .whileTrue(m_climber.moveClimber(-0.5));
    //   }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Autos.autoSideLeft(m_drive, m_arm, m_intake);
  }
}
