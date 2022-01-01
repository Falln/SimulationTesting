// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleConsumer;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


public class RobotContainer {

  DriveSubsystem driveSubsystem;
  XboxController driverController;
  DoubleConsumer doubleConsumer;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    //NetworkTable things


    driveSubsystem = new DriveSubsystem();
    driverController = new XboxController(0);

    driveSubsystem.setDefaultCommand(new TankDriveCommand(
      () -> driverController.getLeftY(),
      () -> driverController.getRightY(),
      driveSubsystem));

    configureButtonBindings();
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
      
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new TankDriveCommand(
      driverController::getLeftY,
      driverController::getRightY,
      driveSubsystem);
  }
}
