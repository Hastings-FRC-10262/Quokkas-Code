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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Revolutions;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class Arm extends SubsystemBase {
  SparkMax armMotorRight, armMotorLeft;
  SparkMaxConfig armMotorRightConfig, armMotorLeftConfig;
  RelativeEncoder encoder;
  PIDController armPID;
  Double armFrontLimit, armRearLimit, armVelocityLimit;
  
  int currentLimit = 30; //stop motor from smoking again, 30V limit

  /** Creates a new Arm. */
  public Arm() {
    armMotorRight = new SparkMax(8, MotorType.kBrushless);
    armMotorLeft = new SparkMax(7, MotorType.kBrushless);

    armMotorRightConfig = new SparkMaxConfig();
    armMotorLeftConfig = new SparkMaxConfig();

    armMotorRight.configure(armMotorRightConfig.
      inverted(false).
      idleMode(IdleMode.kBrake), 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters);

    armMotorRightConfig.smartCurrentLimit(currentLimit);
    //armMotorRightConfig.encoder.positionConversionFactor(1/360.0);
    //armMotorRightConfig.encoder.velocityConversionFactor(1/60.0);
    

    armMotorLeft.configure(armMotorLeftConfig.
      inverted(true).
      follow(armMotorRight, true).
      idleMode(IdleMode.kBrake), 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters);

    armMotorLeftConfig.smartCurrentLimit(currentLimit);

    encoder = armMotorRight.getEncoder();
    encoder.setPosition(ArmConstants.startingAngle);
    

    armPID = new PIDController(ArmConstants.armkP, ArmConstants.armkI, ArmConstants.armkD);
    armPID.setTolerance(ArmConstants.armTolerance);
    }
    
  public Command moveArm(Double velocity) {
    return run(
      () -> {
        armMotorRight.set(velocity);
      }
    );
  }

  public Command moveArmToPosition(Double position) {
    return Commands.run(
        () -> {
          
          // Get the target position, clamped to (limited between) the lowest and highest arm positions
          Double target = MathUtil.clamp(position, ArmConstants.armFrontLimit, ArmConstants.armRearLimit);

          // Calculate the PID result, and clamp to the arm's maximum velocity limit.
          Double result =  MathUtil.clamp(armPID.calculate(encoder.getPosition(), target), -1 * ArmConstants.armVelocityLimit, ArmConstants.armVelocityLimit);

          armMotorRight.set(result);

          
        }).until(() -> armPID.atSetpoint());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm encoder", encoder.getPosition());
    SmartDashboard.putNumber("setpoint", armPID.getSetpoint());
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    
  }
}
