// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class RobotContainer {
  private final DriveTrain m_drive = new DriveTrain();
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake();
  private final Climber m_climber = new Climber();

  // Logitech controller (recognized as a Joystick)
  private final Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Drive (assuming tank drive uses Y-axis of joysticks)
    m_drive.setDefaultCommand(m_drive.driveTank(
      () -> m_driverController.getRawAxis(1), // Left Y-axis
      () -> m_driverController.getRawAxis(5)  // Right Y-axis
    ));

    // *** Arm bindings ***
    new JoystickButton(m_driverController, 4) // Y button
      .onTrue(m_arm.moveArmToPosition(ArmConstants.positionIntakeCoral));

    new JoystickButton(m_driverController, 3) // X button
      .onTrue(m_arm.moveArmToPosition(ArmConstants.positionIntakeAlgae));

    new JoystickButton(m_driverController, 2) // B button
      .onTrue(m_arm.moveArmToPosition(ArmConstants.positionRemoveAlgaeLow));

    new JoystickButton(m_driverController, 1) // A button
      .onTrue(m_arm.moveArmToPosition(ArmConstants.positionRemoveAlgaeHigh));

    // D-Pad bindings for arm movement
    new POVButton(m_driverController, 180) // D-Pad Down
      .onTrue(m_arm.moveArmToPosition(ArmConstants.positionClimbStart));

    new POVButton(m_driverController, 0) // D-Pad Up
      .onTrue(m_arm.moveArmToPosition(ArmConstants.positionClimbEnd));

    // *** Intake bindings ***
    m_intake.setDefaultCommand(m_intake.moveIntake(0.0));

    new JoystickButton(m_driverController, 6) // Right bumper
      .whileTrue(m_intake.moveIntake(0.75));

    new JoystickButton(m_driverController, 8) // Right trigger (mapped as a button)
      .whileTrue(m_intake.moveIntake(-0.75));

    // *** Climber bindings ***
    m_climber.setDefaultCommand(m_climber.moveClimber(0.0));

    new JoystickButton(m_driverController, 5) // Left bumper
      .whileTrue(m_climber.moveClimber(0.5));

    new JoystickButton(m_driverController, 7) // Left trigger (mapped as a button)
      .whileTrue(m_climber.moveClimber(-0.5));
  }

  public Command getAutonomousCommand() {
    return Autos.autoSideLeft(m_drive, m_arm, m_intake);
  }
}