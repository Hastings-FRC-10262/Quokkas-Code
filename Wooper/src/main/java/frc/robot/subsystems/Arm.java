// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.ArmConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class Arm extends SubsystemBase {
  SparkMax armMotor1, armMotor2;
  SparkMaxConfig armMotor1Config, armMotor2Config;
  RelativeEncoder encoder;
  PIDController armPID;
  Double armFrontLimit, armRearLimit, armVelocityLimit;
  
  int currentLimit = 30; //stop motor from smoking again, 30V limit

  /** Creates a new Arm. */
  public Arm() {
    armMotor1 = new SparkMax(8, MotorType.kBrushless);
    armMotor2 = new SparkMax(7, MotorType.kBrushless);

    armMotor1Config = new SparkMaxConfig();
    armMotor2Config = new SparkMaxConfig();

    armMotor1.configure(armMotor1Config.
      //inverted(true).
      idleMode(IdleMode.kBrake), 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters);

    armMotor1Config.smartCurrentLimit(currentLimit);

    armMotor2.configure(armMotor2Config.
      inverted(true).
      follow(armMotor1, true).
      idleMode(IdleMode.kBrake), 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters);

    armMotor2Config.smartCurrentLimit(currentLimit);

    encoder = armMotor1.getEncoder();
    

    armPID = new PIDController(ArmConstants.armkP, ArmConstants.armkI, ArmConstants.armkD);

  }


  public Command moveArm(Double velocity) {
    return run(
      () -> {

        armMotor1.set(velocity);
        System.out.println("moveArm");
      }
    );
  }

  public Command moveArmToPosition(Double position) {
    return run(
        () -> {
          
          // Get the target position, clamped to (limited between) the lowest and highest arm positions
          Double target = MathUtil.clamp(position, ArmConstants.armRearLimit, ArmConstants.armFrontLimit);

          // Calculate the PID result, and clamp to the arm's maximum velocity limit.
          Double result =  MathUtil.clamp(armPID.calculate(encoder.getPosition(), target), -1 * ArmConstants.armVelocityLimit, ArmConstants.armVelocityLimit);

          armMotor1.set(result);

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
    SmartDashboard.putNumber("arm encoder raw", encoder.getPosition());
    //SmartDashboard.putBoolean("setpoint", e)
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
