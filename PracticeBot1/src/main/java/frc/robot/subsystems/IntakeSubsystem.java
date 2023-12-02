// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private PWMVictorSPX m_motor = new PWMVictorSPX(IntakeConstants.IntakeCANID);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {


  }

  public void setPower(double power){
    m_motor.set(power);
  }

  public void hold(){
    setPower(IntakeConstants.holdNumber);
  }

  public void stopMotor(){
    m_motor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
