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
import frc.robot.commands.*;
import frc.robot.subsystems.*;


public class RobotContainer {

  class PathWeaverData {
    public String JSON;
    public Trajectory trajectory;
    public CustomRamseteCommand ramseteCommand;
    public PathWeaverData(String pathWeaverJSON) {JSON = pathWeaverJSON;}
  }

  DriveSubsystem driveSubsystem;
  XboxController driverController;

  //PathWeaverJSONs
  PathWeaverData testPath1 = new PathWeaverData("output/testPath1.wpilib.json");
  PathWeaverData testPath2 = new PathWeaverData("output/testPath2.wpilib.json");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    //NetworkTable things

    //Instantiate subsystems
    driveSubsystem = new DriveSubsystem();

    //Instantiate controllers and Triggers 
    driverController = new XboxController(0);

    //Set the default commands of subsystems
    driveSubsystem.setDefaultCommand(new TankDriveCommand(
      () -> driverController.getLeftY(),
      () -> driverController.getRightY(),
      driveSubsystem));

    loadPathWeaverTrajectories(testPath1, testPath2);
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

  private void loadPathWeaverTrajectories(PathWeaverData... pathWeaverData) {
    for (PathWeaverData path:pathWeaverData) {
      path.trajectory = getAutoTrajectoryFromPathWeaverJSON(path.JSON);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    driveSubsystem.resetEncoders();
    driveSubsystem.resetGyro();
    driveSubsystem.resetPose(testPath1.trajectory.getInitialPose());
    return new CustomRamseteCommand(testPath1.trajectory, driveSubsystem)
           .andThen(new CustomRamseteCommand(testPath2.trajectory, driveSubsystem));
  }

  private Trajectory getAutoTrajectoryFromPathWeaverJSON(String pathWeaverJSON) {
    try {
      File coreFile = new File("C:\\Users\\isaac\\Documents\\Program Projects\\FRC\\SimulationTestingParent\\SimulationTesting\\PathWeaver");
      Path trajectoryPath = coreFile.toPath().resolve(pathWeaverJSON);
      return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + pathWeaverJSON, ex.getStackTrace());
      return TrajectoryGenerator.generateTrajectory(
        driveSubsystem.getPose(),
        new ArrayList<>(),
        driveSubsystem.getPose(),
        ConstantsPW.autoTrajectoryConfig);
    }
  }
}
