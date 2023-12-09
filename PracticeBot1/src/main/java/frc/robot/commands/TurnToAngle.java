// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAngle extends PIDCommand {
  /** Creates a new TurnToAngle. */

  DriveSubsystem m_driveSubsystem;

  public TurnToAngle(DriveSubsystem driveSubsystem, double targetAngle) {

// ku = 0.17
// tu = 0.1

// kp = 0.6 * ku
// ki = (1.2 * ku) / tu
// kd = ku * tu * 0.075

    super(
        // The controller that the command will use
        new PIDController(0.102, 2.04, 0.001275),
        // This should return the measurement
        driveSubsystem::getHeading,
        // This should return the setpoint (can also be a constant)
        targetAngle,
        // This uses the output
        output -> {
          // Use the output here
          driveSubsystem.turn(output);
        });

        getController().setTolerance(0.01, 1);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // public double getCorrectHeading(double targetAngle){

  //   if(targetAngle > 170 || targetAngle < -170){
  //     return 
  //   }


  // }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}