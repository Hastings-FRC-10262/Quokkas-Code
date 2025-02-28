// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.ArmConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.PWM;

public class Arm extends SubsystemBase {
  PWMSparkMax armMotorRight, armMotorLeft;
  //SparkMaxConfig armMotorRightConfig, armMotorLeftConfig;
  DutyCycleEncoder encoder;
  PIDController armP;
  Double armFrontLimit, armRearLimit, armVelocityLimit;
  

  /** Creates a new Arm. */
  public Arm() {
    armMotorRight = new PWMSparkMax(5);
    armMotorLeft = new PWMSparkMax(4);

    //look into PWMSparkMax library

    //armMotorRightConfig = new SparkMaxConfig();
    //armMotorLeftConfig = new SparkMaxConfig();

    armMotorRight.setSafetyEnabled(true);
    armMotorLeft.setSafetyEnabled(true);
    armMotorRight.setInverted(true);
    armMotorRight.addFollower(armMotorLeft);


    

    // armMotorRight.configure(armMotorRightConfig.
    //   inverted(true).
    //   idleMode(IdleMode.kBrake), 
    //   ResetMode.kNoResetSafeParameters, 
    //   PersistMode.kPersistParameters);

    // armMotorLeft.configure(armMotorLeftConfig.
    //   follow(armMotorRight, true).
    //   idleMode(IdleMode.kBrake), 
    //   ResetMode.kNoResetSafeParameters, 
    //   PersistMode.kPersistParameters);

    encoder = new DutyCycleEncoder(5); //CHECK PORT!!
    armP = new PIDController(ArmConstants.armkP, ArmConstants.armkI, ArmConstants.armkD);

  }

  public Command moveArm(Double velocity){
    return run(
      () -> {
        Double result =  MathUtil.clamp(velocity, -1 * ArmConstants.armVelocityLimit, ArmConstants.armVelocityLimit);

        

        armMotorRight.set(result);
      }
    );
  }

  public Command moveArmToPosition(Double position) {
    return run(
        () -> {
          
          // Get the target position, clamped to (limited between) the lowest and highest arm positions
          Double target = MathUtil.clamp(position, ArmConstants.armRearLimit, ArmConstants.armFrontLimit);

          // Calculate the PID result, and clamp to the arm's maximum velocity limit.
          Double result =  MathUtil.clamp(armP.calculate(encoder.get(), target), -1 * ArmConstants.armVelocityLimit, ArmConstants.armVelocityLimit);

          armMotorRight.set(result);

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
    SmartDashboard.putNumber("encoder arm", encoder.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
