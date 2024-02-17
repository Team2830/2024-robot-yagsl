// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Intake extends SubsystemBase {

    private CANSparkMax topIntake = new CANSparkMax(13, MotorType.kBrushed);
    private CANSparkMax bottomIntake = new CANSparkMax(14, MotorType.kBrushed);

    public Intake(){
        topIntake.setInverted(true);
        bottomIntake.setInverted(true);
        topIntake.setSmartCurrentLimit(80);
        bottomIntake.setSmartCurrentLimit(80);  
    }

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
  
  public void setSpeed(double speed){
    topIntake.set(speed);
    bottomIntake.set(speed);
  }

   
}