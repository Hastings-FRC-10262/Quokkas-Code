// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class Intake extends SubsystemBase {
  SparkMax intakeMotorRight, intakeMotorLeft;
  SparkMaxConfig intakeMotorRightConfig, intakeMotorLeftConfig;


  /** Creates a new Intake. */
  public Intake() {
    intakeMotorRight = new SparkMax(7, MotorType.kBrushed);
    intakeMotorLeft = new SparkMax(6, MotorType.kBrushed);

    intakeMotorRightConfig = new SparkMaxConfig();
    intakeMotorLeftConfig = new SparkMaxConfig();


    intakeMotorRight.configure(intakeMotorRightConfig.
      inverted(false).
      idleMode(IdleMode.kBrake), 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters);

    intakeMotorLeft.configure(intakeMotorLeftConfig.
      idleMode(IdleMode.kBrake).
      follow(7), 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters);


  }

  public Command moveIntake(Double velocity) {
    // Inline construction of command goes here.
    return run(
        () -> {
          
          intakeMotorRight.set(velocity);
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
