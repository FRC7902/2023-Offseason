// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDistance extends CommandBase {

  private final DriveSubsystem m_DriveSubsystem;
  private final double targetDistance;

//ku = 19.5
//tu = 0.22

  private final PIDController drivePID = new PIDController(11.5, 106.36, 0.32175);
  private double initialPositionX;
  private double angle;
  /** Creates a new DriveToDistance. */
  public DriveToDistance(DriveSubsystem drive, double distance) {
    m_DriveSubsystem = drive;
    targetDistance = distance;
    m_DriveSubsystem.resetEncoders();
    drivePID.setTolerance(0.1, 0.1);
    addRequirements(drive);
    initialPositionX = m_DriveSubsystem.getDisplacementX();
    angle = m_DriveSubsystem.getHeading();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPositionX = m_DriveSubsystem.getDisplacementX();
    angle = m_DriveSubsystem.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = drivePID.calculate(m_DriveSubsystem.getDisplacementX(), initialPositionX + targetDistance * Math.cos(angle));
    m_DriveSubsystem.driveRaw(speed);
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
