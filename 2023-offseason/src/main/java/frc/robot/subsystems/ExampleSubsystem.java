// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {

  private final CANSparkMax m_motor1 = new CANSparkMax(13, MotorType.kBrushless);
  private final CANSparkMax m_motor2 = new CANSparkMax(14, MotorType.kBrushless);
  private final CANSparkMax m_motor3 = new CANSparkMax(15, MotorType.kBrushless);
  private final CANSparkMax m_motor4 = new CANSparkMax(16, MotorType.kBrushless);

  private final MotorControllerGroup left = new MotorControllerGroup(m_motor1, m_motor2);
  private final MotorControllerGroup right = new MotorControllerGroup(m_motor3, m_motor4);
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    //left.setInverted(true);
    m_motor1.setInverted(true);
    m_motor2.setInverted(true);
    m_motor3.setInverted(false);
    m_motor4.setInverted(false);

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }


  public void setPower(double power) {
    left.set(power);
    right.set(power);

    // if statements needed for testing
  }

  public void stopMotor(){
    left.set(0);
    right.set(0);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
