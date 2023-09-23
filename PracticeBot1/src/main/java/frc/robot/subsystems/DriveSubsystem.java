// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveSubsystem extends SubsystemBase {

  //Declare motor controllers
  private final CANSparkMax m_leftleader = new CANSparkMax(13, MotorType.kBrushless);
  private final CANSparkMax m_leftfollower = new CANSparkMax(14, MotorType.kBrushless);
  private final CANSparkMax m_rightleader = new CANSparkMax(15, MotorType.kBrushless);
  private final CANSparkMax m_rightfollower = new CANSparkMax(16, MotorType.kBrushless);

  private final MotorControllerGroup left = new MotorControllerGroup(m_leftleader, m_leftfollower);
  private final MotorControllerGroup right = new MotorControllerGroup(m_rightleader, m_rightfollower);
  private final DifferentialDrive drive = new DifferentialDrive(left, right);

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    left.setInverted(true);
    right.setInverted(false);


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

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public void driveArcade(double xForward, double zRotation){
    drive.arcadeDrive(xForward, zRotation);
  }

  public void driveRaw(double power){
    left.set(power);
    right.set(power);
  }

  public void turnLeft(double amount){
    left.set(-amount);
    right.set(amount);
  }

  public void turnRight(double amount){
    left.set(amount);
    right.set(-amount);
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
