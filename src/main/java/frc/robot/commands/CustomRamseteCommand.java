// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveSubsystem;

public class CustomRamseteCommand extends RamseteCommand {
  /** Creates a new RamseteCommand. */
  public CustomRamseteCommand(Trajectory trajectory, boolean reversed, DriveSubsystem driveSubsystem) {
    super();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public CustomRamseteCommand() {
    super();
  }
}
