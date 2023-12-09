// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngle extends CommandBase {

  private final DriveSubsystem m_driveSubsystem;
  private final double targetAngle;

  private final PIDController turnPID = new PIDController(0.102, 2.04, 0.001275);

  private boolean isAdditive;
  private double initialAngle;
  private int direction;
  /** Creates a new TurnToAngleB. */
  public TurnToAngle(DriveSubsystem drive, double angle, boolean IsAdditive) {

    m_driveSubsystem = drive;
    targetAngle = angle;
    isAdditive = IsAdditive;
    turnPID.setTolerance(0.01, 1);


    initialAngle = m_driveSubsystem.getHeadingCase2();

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  initialAngle = m_driveSubsystem.getHeadingCase2();

  double trueAngle = initialAngle - targetAngle;

    while(trueAngle < 0){
      trueAngle += 360;
    }

    while(trueAngle > 360){
      trueAngle -= 360;
    }

  if(trueAngle < 180){
    direction = 1;
  }else{
    direction = 1;
  }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed;


    SmartDashboard.putNumber("targetAngle", (initialAngle + targetAngle)%360);


    if(isAdditive){
      if((initialAngle + targetAngle)%360 > 340 || (initialAngle + targetAngle)%360 < 20){
        speed = turnPID.calculate(m_driveSubsystem.getHeading(), (initialAngle + targetAngle)%360);
        m_driveSubsystem.turn(direction * speed);
      }else{
        speed = turnPID.calculate(m_driveSubsystem.getHeadingCase2(), initialAngle + targetAngle);
        m_driveSubsystem.turn(direction * speed);
      }
    }else{
      if(targetAngle > 340 || targetAngle < 20){
        speed = turnPID.calculate(m_driveSubsystem.getHeading(), targetAngle);
        m_driveSubsystem.turn(direction * speed);

      }else{
        speed = turnPID.calculate(m_driveSubsystem.getHeadingCase2(), targetAngle);
        m_driveSubsystem.turn(direction * speed);

      }
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turnPID.atSetpoint();
  }
}
