// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class DriveTrain extends SubsystemBase {
  Spark leftFront, leftRear, rightFront, rightRear;
  //SparkMaxConfig leftFrontConfig, leftRearConfig, rightFrontConfig, rightRearConfig;


  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftFront = new Spark(0);
    leftRear = new Spark(1);
    rightFront = new Spark(2);
    rightRear = new Spark(3);

    leftFront.setInverted(true);
    //leftFront.setSafetyEnabled(true);
    leftFront.addFollower(leftRear);

    //leftRear.setSafetyEnabled(true);


    rightFront.setInverted(true);
    //rightFront.setSafetyEnabled(true);
    rightFront.addFollower(rightRear);

    //rightRear.setSafetyEnabled(true);

    // leftFrontConfig = new SparkMaxConfig();
    // leftRearConfig = new SparkMaxConfig();
    // rightFrontConfig = new SparkMaxConfig();
    // rightRearConfig = new SparkMaxConfig();

    // leftFront.configure(leftFrontConfig.
    //   inverted(true).
    //   idleMode(IdleMode.kBrake), 
    //   ResetMode.kNoResetSafeParameters, 
    //   PersistMode.kPersistParameters);

    // leftRear.configure(leftRearConfig.
    //   idleMode(IdleMode.kBrake).
    //   follow(0),
    //   ResetMode.kNoResetSafeParameters, 
    //   PersistMode.kPersistParameters);

    

    // rightFront.configure(rightFrontConfig.
    //   inverted(false).
    //   idleMode(IdleMode.kBrake), 
    //   ResetMode.kNoResetSafeParameters, 
    //   PersistMode.kPersistParameters);

    // rightRear.configure(rightRearConfig.
    //   idleMode(IdleMode.kBrake).
    //   follow(2),
    //   ResetMode.kNoResetSafeParameters, 
    //   PersistMode.kPersistParameters);

  }


 
  public Command driveTank(DoubleSupplier left, DoubleSupplier right) {
    // Inline construction of command goes here.
    return run(
        () -> {
          
          leftFront.set(-left.getAsDouble());
          rightFront.set(right.getAsDouble());
        });
  }

  public Command moveStraight(Double velocity) {
    return run(
      () -> {
        leftFront.set(velocity);
        rightFront.set(velocity);
      });
  }

  public Command turn(Double velocity) {
    return run(
      () -> {
        leftFront.set(velocity);
        rightFront.set(-1 * velocity);
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
