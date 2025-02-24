// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Encoder;


public class DriveTrain extends SubsystemBase {
  SparkMax leftFront, leftRear, rightFront, rightRear;
  SparkMaxConfig leftFrontConfig, leftRearConfig, rightFrontConfig, rightRearConfig;


  private final Encoder leftEncoder = new Encoder(0,0);
  private final Encoder rightEncoder = new Encoder(0, 0);

  public double getLeftEncoderPosition() {
    return leftEncoder.get();
  }

  public double getRightEncoderPosition() {
    return rightEncoder.get();
  }

  public double getLeftEncoderVelocity() {
    return leftEncoder.getRate(); // Returns counts per second
  }

  public double getRightEncoderVelocity() {
    return rightEncoder.getRate();
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftFront = new SparkMax(4, MotorType.kBrushless);
    leftRear = new SparkMax(3, MotorType.kBrushless);
    rightFront = new SparkMax(2, MotorType.kBrushless);
    rightRear = new SparkMax(1, MotorType.kBrushless);

    leftFrontConfig = new SparkMaxConfig();
    leftRearConfig = new SparkMaxConfig();
    rightFrontConfig = new SparkMaxConfig();
    rightRearConfig = new SparkMaxConfig();

    leftFront.configure(leftFrontConfig.
      inverted(true).
      idleMode(IdleMode.kBrake), 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters);

    leftRear.configure(leftRearConfig.
      idleMode(IdleMode.kBrake).
      follow(4),
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters);

    rightFront.configure(rightFrontConfig.
      inverted(false).
      idleMode(IdleMode.kBrake), 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters);

    rightRear.configure(rightRearConfig.
      idleMode(IdleMode.kBrake).
      follow(2),
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters);

  }


 
  public Command driveTank(DoubleSupplier left, DoubleSupplier right) {
    // Inline construction of command goes here.
    return run(
        () -> {
          
          leftFront.set(left.getAsDouble());
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

  public Command moveStraightPID(double feet, double velocity) {
    return new Command() {
        private double targetDistance;

        @Override
        public void initialize() {
            
            resetEncoders();

            
            double wheelCircumference = Math.PI * 6.0;
            double feetPerRevolution = wheelCircumference / 12.0;
            double countsPerFoot = 2048.0 / feetPerRevolution;

            targetDistance = feet * countsPerFoot;
        }

        @Override
        public void execute() {
            leftFront.set(velocity);
            rightFront.set(velocity);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(getLeftEncoderPosition()) >= targetDistance ||
                   Math.abs(getRightEncoderPosition()) >= targetDistance;
        }

        @Override
        public void end(boolean interrupted) {
            
            leftFront.set(0);
            rightFront.set(0);
        }
    };
}

public Command turnDegreesPID(double degrees, double velocity) {
  return new Command() {
      private double targetDistance;
      private final double wheelBaseWidth = 24.0;
      private final double wheelCircumference = Math.PI * 6.0;

      @Override
      public void initialize() {
          
          resetEncoders();

          
          double robotCircumference = Math.PI * wheelBaseWidth;
          double feetPerDegree = (robotCircumference / 12.0) / 360.0;
          double feetToTravel = feetPerDegree * degrees;
          double countsPerFoot = 2048.0 / (wheelCircumference / 12.0);

          targetDistance = feetToTravel * countsPerFoot;
      }

      @Override
      public void execute() {
          leftFront.set(velocity);
          rightFront.set(-velocity);
      }

      @Override
      public boolean isFinished() {
          
          return Math.abs(getLeftEncoderPosition()) >= targetDistance ||
                 Math.abs(getRightEncoderPosition()) >= targetDistance;
      }

      @Override
      public void end(boolean interrupted) {
          // Stop the motors when finished
          leftFront.set(0);
          rightFront.set(0);
      }
  };
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
