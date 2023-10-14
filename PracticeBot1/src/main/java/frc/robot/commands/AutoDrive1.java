// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public final class AutoDrive1 extends SequentialCommandGroup {

  public AutoDrive1(DriveSubsystem drive) {
    addCommands(
        new driveForward(drive).withTimeout(1),
        new turnLeft(drive).withTimeout(1),
        new driveBackward(drive).withTimeout(1),
        new turnRight(drive).withTimeout(1));

  }
}
