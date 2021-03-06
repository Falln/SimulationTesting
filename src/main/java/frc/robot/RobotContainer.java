// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


public class RobotContainer {

  /**
   * This class simply allows us to store the PathWeaver JSON name and the 
   * trajectory associated with it in the same place (and possibly the 
   * command created from it too - not used currently) 
   */
  class PathWeaverData {
    public String JSONName;
    public Trajectory trajectory;
    public Command command;
    public PathWeaverData(String pathWeaverJSON) {JSONName = pathWeaverJSON;}
  }

  //Subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  //Controllers and Triggers
  private final XboxController driverController = new XboxController(0);

  //PathWeaverJSONs
  PathWeaverData testPath1 = new PathWeaverData("testPath1");
  PathWeaverData testPath2 = new PathWeaverData("testPath2");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Silence the "Missing Joystick" warnings
    DriverStation.silenceJoystickConnectionWarning(true);
    
    //NetworkTable things

    //Configure the Button/Trigger bindings
    configureButtonBindings();

    //Set the default commands of subsystems
    driveSubsystem.setDefaultCommand(new TankDriveCommand(
      () -> driverController.getLeftY(),
      () -> driverController.getRightY(),
      driveSubsystem));

    //Load all paths
    loadPathWeaverTrajectories(testPath1, testPath2);
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
   * Takes the given PathWeaverDatas, generates the trajectory assoicated with the
   * JSONName they have, and then sets the PathWeaverData's trajectory to that trajectory
   * @param pathWeaverData PathWeaverData class to load trajectories to
   */
  private void loadPathWeaverTrajectories(PathWeaverData... pathWeaverData) {
    for (PathWeaverData pwData:pathWeaverData) {
      pwData.trajectory = driveSubsystem.loadTrajectoryFromPWJSON((pwData.JSONName));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //Generate the path commands
    Command path1Command = driveSubsystem.createCommandFromTrajectory(testPath1.trajectory, true);
    Command path2Command = driveSubsystem.createCommandFromTrajectory(testPath2.trajectory);

    //Return the combined command
    return path1Command.andThen(new WaitCommand(2)).andThen(path2Command);
  }


}
