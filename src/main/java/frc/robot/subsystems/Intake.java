// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Intake extends SubsystemBase {

    private WPI_TalonFX topIntake = new WPI_TalonFX(13);
    private WPI_TalonFX bottomIntake = new WPI_TalonFX(14 );

    public Intake(){
        topIntake.setInverted(false);
        bottomIntake.setInverted(false);
        // topIntake.setSmartCurrentLimit(140);
        // bottomIntake.setSmartCurrentLimit(140);  
    }

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
  
  public void setSpeed(double speed){
    topIntake.set(speed);
    bottomIntake.set(speed);
  }

  public void setOutput(double voltage){
    topIntake.setVoltage(voltage);
    bottomIntake.setVoltage(voltage);
  }
   
}
