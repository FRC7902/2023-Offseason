// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDistance extends CommandBase {

  private final DriveSubsystem m_DriveSubsystem;
  private final double targetDistance;

//ku = 10.7
//tu = 0.33

// kp = 0.6 * ku
// ki = (1.2 * ku) / tu
// kd = ku * tu * 0.075
  private final PIDController drivePID = new PIDController(6.42, 38.91, 0.264825);
  private double initialPositionX;
  private double initialPositionY;
  private double angle;
  /** Creates a new DriveToDistance. */
  public DriveToDistance(DriveSubsystem drive, double distance) {
    m_DriveSubsystem = drive;
    targetDistance = distance;
    m_DriveSubsystem.resetEncoders();
    drivePID.setTolerance(0.01, 1);
    addRequirements(drive);
    initialPositionX = m_DriveSubsystem.getDisplacementX();
    initialPositionY = m_DriveSubsystem.getDisplacementY();
    angle = m_DriveSubsystem.getHeadingCase2();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPositionX = m_DriveSubsystem.getDisplacementX();
    initialPositionY = m_DriveSubsystem.getDisplacementY();
    angle = m_DriveSubsystem.getHeadingCase2();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed;

    if((45 <  angle && angle < 135) || (225 <  angle && angle < 315)){
      speed = drivePID.calculate(m_DriveSubsystem.getDisplacementY(), initialPositionY + (targetDistance * Math.sin(Math.toRadians(angle))));
      if(Math.sin(Math.toRadians(angle)) < 0){
        m_DriveSubsystem.driveRaw(-speed);
      }else{
        m_DriveSubsystem.driveRaw(speed);
      }
    }else{
      speed = drivePID.calculate(m_DriveSubsystem.getDisplacementX(), initialPositionX + (targetDistance * Math.cos(Math.toRadians(angle))));
      if(Math.cos(Math.toRadians(angle)) < 0){
        m_DriveSubsystem.driveRaw(-speed);
      }else{
        m_DriveSubsystem.driveRaw(speed);
      }
    }
    

    SmartDashboard.putNumber("target", initialPositionX + (targetDistance * Math.cos(Math.toRadians(angle))));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivePID.atSetpoint();
  }
}
