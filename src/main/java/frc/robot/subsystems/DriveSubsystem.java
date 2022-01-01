package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.ConstantsPW;

import java.io.File;
import java.io.IOException;
import java.lang.System.Logger;
import java.nio.file.FileSystem;
import java.nio.file.Path;
import java.nio.file.Paths;

import javax.naming.directory.DirContext;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimValue;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.CustomRamseteCommand;

public class DriveSubsystem extends SubsystemBase {

  CANSparkMax leftSpark;
  CANSparkMax rightSpark;

  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;

  AHRS navX;
  SimDeviceSim navXSim;

  DifferentialDrive drive;
  DifferentialDrivetrainSim driveSim;

  //Way of keeping track of volts since SparkMAX currently doesnt
  double voltsSuppliedLeft = 0;
  double voltsSuppliedRight = 0;

  Field2d field2d;
  DifferentialDriveOdometry odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    //Instantiate motor controllers and reverse them as reverse is the true robot front
    leftSpark = new CANSparkMax(4, MotorType.kBrushless);
    rightSpark = new CANSparkMax(5, MotorType.kBrushless);

    if (Robot.isSimulation()) {
      leftSpark.setInverted(true);
      rightSpark.setInverted(true);
    } else {
      leftSpark.setInverted(true);
      rightSpark.setInverted(true);
    }

    //Get the RelativeEncoder object from the sparks 
    leftEncoder = leftSpark.getEncoder();
    rightEncoder = rightSpark.getEncoder();
    //TODO figure out correct conversion factor. NOTE must be in meters
    leftEncoder.setPositionConversionFactor(Constants.driveEncConversionFactor);
    rightEncoder.setPositionConversionFactor(Constants.driveEncConversionFactor);

    //Instantiate the navX and reset it as we're booting up
    navX = new AHRS(SPI.Port.kMXP);
    navX.reset();

    //Instantiate the odometry object and the field2D
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()));
    field2d = new Field2d();
    SmartDashboard.putData("Field", field2d);

    //Instantiate the drivetrain
    drive = new DifferentialDrive(leftSpark, rightSpark);
    drive.setDeadband(0.07); //Same thing as setting a min input value but a little different

    //Things needed for drive simulation
    if (Robot.isSimulation()) {
      driveSim = DifferentialDrivetrainSim.createKitbotSim(
        KitbotMotor.kDoubleNEOPerSide, // 2 CIMs per side.
        KitbotGearing.k10p71,        // 10.71:1
        KitbotWheelSize.kSixInch,     // 6" diameter wheels.
        null                         // No measurement noise.
        );
    }
  }

  /** Set the power of the left and right sides of the drivetrain independently */
  public void driveTank(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }
  
  /** Drive the drivetrain similar to a videogame 1st person shooter */
  public void driveArcade(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  }

  /** Sets the drivetrains speed in volts rather than based on a duty-cycle */
  public void driveTankVolts(double leftVolts, double rightVolts) {
    leftSpark.setVoltage(leftVolts);
    rightSpark.setVoltage(-rightVolts);
    voltsSuppliedLeft = leftVolts;
    voltsSuppliedRight = rightVolts;
    drive.feed();
  }

  /** Stop all drive motors */
  public void stopDrive() {
    drive.stopMotor();
    voltsSuppliedLeft = 0;
    voltsSuppliedRight = 0;
  }

  /** Returns the distance in meters the left encoder has traveled */
  public double getLeftEncoderDistance() {
    return leftEncoder.getPosition();
  }

  /** Returns the distance in meters the right encoder has traveled */
  public double getRightEncoderDistance() {
    return rightEncoder.getPosition();
  }

  /** Returns the avergae distance in meters both encoders have traveled */
  public double getAveEncDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance())/2;
  }

  /** Resets the postion of both encoders to 0 */
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    if (Robot.isSimulation()) {
      setSimDoubleFromDeviceData("SPARK MAX [4]", "Position", 0);
      setSimDoubleFromDeviceData("SPARK MAX [5]", "Position", 0);
    }
  }

  /** Returns a DiffDriveWheelSpeeds object based on the left and right encoder veloctiy in m/s */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
  }

  /** Gets the angle from gyro in degrees. Note this is the yaw of the navX */
  public double getAngle() {
    return navX.getAngle();
  }

  /** Resets all values of the gyro */
  public void resetGyro() {
    navX.reset();

    if (Robot.isSimulation()) {
      setSimDoubleFromDeviceData("navX-Sensor[0]", "Yaw", 0);
    }
  }

  /** 
   * Gets the current calculated pose of the robot. This is determined by the driveOdometry that is 
   * updated in the periodic loop
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Updates the drive odometry */
  public void updateOdometry() {
    odometry.update(Rotation2d.fromDegrees(getAngle()), leftEncoder.getPosition(), rightEncoder.getPosition());
  }

  /**
   * Resets the odometry to the specified pose.
   * 
   * @param pose The pose to which to set the odometry.
   */
  public void setPose(Pose2d startingPose) {
    resetEncoders();
    //TODO might possibly need a -getAngle() call
    odometry.resetPosition(startingPose, Rotation2d.fromDegrees(getAngle()));
    if (Robot.isSimulation()) {
      driveSim.setPose(startingPose);
    }
  }

  public void setSimPose(Pose2d startingPose) {
    driveSim.setPose(startingPose);
  }





  public Trajectory loadTrajectoryFromPWJSON(String pathWeaverJSONName) {
    try {
      var filePath = Filesystem.getDeployDirectory().toPath().resolve(Paths.get("output", pathWeaverJSONName + ".wpilib.json"));
      return TrajectoryUtil.fromPathweaverJson(filePath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + pathWeaverJSONName, ex.getStackTrace());
      return new Trajectory();
    }
  }

  public Command createCommandFromTrajectory(Trajectory trajectory, boolean initPose) {
    if (initPose) {
      return new InstantCommand(() -> this.setPose(trajectory.getInitialPose()))
        .andThen(new CustomRamseteCommand(trajectory, this));
    }
    return new CustomRamseteCommand(trajectory, this);
    }

  public Command createCommandFromTrajectory(Trajectory trajectory) {
      return new CustomRamseteCommand(trajectory, this);
  }





  @Override
  public void periodic() {
    //This method will be called once per scheduler run
    updateOdometry();
    field2d.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putNumber("Robot x", odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Robot y", odometry.getPoseMeters().getTranslation().getY());
  }

  @Override
  public void simulationPeriodic() {
    //NOTE get() needs to be - as our motor controllers are inverted (inversion happens on output, not on set()/get())
    // driveSim.setInputs(-leftSpark.get() * RobotController.getInputVoltage(),
    // -rightSpark.get() * RobotController.getInputVoltage());
    driveSim.setInputs(voltsSuppliedLeft, voltsSuppliedRight);

    driveSim.update(0.02);

    leftSpark.get();
    //update navX and Spark data (as much as needed)
    setSimDoubleFromDeviceData("navX-Sensor[0]", "Yaw", driveSim.getHeading().getDegrees());
    setSimDoubleFromDeviceData("SPARK MAX [4]", "Position", driveSim.getLeftPositionMeters());
    setSimDoubleFromDeviceData("SPARK MAX [5]", "Position", driveSim.getRightPositionMeters());
    setSimDoubleFromDeviceData("SPARK MAX [4]", "Applied Output", voltsSuppliedLeft);
    setSimDoubleFromDeviceData("SPARK MAX [5]", "Applied Output", voltsSuppliedRight);
  }
  

  /** 
   * Allows for shorter more direct references of SimDoubles. Takes a SimDevice device name,
   * a key that points to a SimDouble, and a double to set the SimDouble to.
   * 
   * @param deviceName The name of the SimDevice
   * @param keyName The key that the double is associated with'
   * @param value Double that will be set to the SimDouble at the given key
   */
  public void setSimDoubleFromDeviceData(String deviceName, String keyName, double value) {
    SimDouble simDouble = new SimDouble(
      SimDeviceDataJNI.getSimValueHandle(
          SimDeviceDataJNI.getSimDeviceHandle(deviceName),
          keyName));
    simDouble.set(value);
  }
}
