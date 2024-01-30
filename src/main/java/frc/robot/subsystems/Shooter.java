// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax topLeft = new CANSparkMax(9, MotorType.kBrushless);
  private CANSparkMax bottomLeft = new CANSparkMax(10, MotorType.kBrushless);
  private CANSparkMax topRight = new CANSparkMax(11, MotorType.kBrushless);
  private CANSparkMax bottomRight = new CANSparkMax(12, MotorType.kBrushless);
  public Shooter() {
    topLeft.setInverted(false);
    topRight.setInverted(true);
    bottomLeft.setInverted(false);
    bottomRight.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

    public void setTopMotorSpeed(double speed){
    topLeft.set(speed);
    topRight.set(speed);

    }

    public void setBottomMotorSpeed(double speed){
      bottomLeft.set(speed);
      bottomRight.set(speed);
    }
  }
