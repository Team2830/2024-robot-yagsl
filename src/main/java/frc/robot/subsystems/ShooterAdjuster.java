// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterAdjuster extends SubsystemBase {
  CANSparkMax motor = new CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless);

  /** Creates a new ShooterAdjuster. */
  public ShooterAdjuster() {
    motor.getPIDController().setFeedbackDevice(motor.getAbsoluteEncoder());
    motor.getPIDController().setP(0);
    motor.getPIDController().setI(0);
    motor.getPIDController().setD(0);
    motor.getAbsoluteEncoder().setPositionConversionFactor(360);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPosition(double angle) {
    motor.getPIDController().setReference(angle, CANSparkBase.ControlType.kPosition);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }
}
