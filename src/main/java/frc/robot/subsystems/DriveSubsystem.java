package frc.robot.subsystems;

import java.security.spec.ECFieldF2m;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import javax.swing.event.DocumentEvent.ElementChange;

import com.fasterxml.jackson.databind.util.LRUMap;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  //2 spark 2 encoders 1 gyro
  Spark leftDriveSpark;
  Spark rightDriveSpark;

  Encoder leftEncoder; 
  Encoder rightEncoder;

  DifferentialDrive drive;

  AHRS navX;

  Field2d field2d;
  DifferentialDriveOdometry odometry;


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    //Initalize different components
    leftDriveSpark = new Spark(0);
    rightDriveSpark = new Spark(1);

    drive = new DifferentialDrive(leftDriveSpark, rightDriveSpark);

    leftEncoder = new Encoder(0,1);
    rightEncoder = new Encoder(2,3);
    leftEncoder.reset();
    rightEncoder.reset();

    navX = new AHRS(SPI.Port.kMXP);
    navX.reset();

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()));
    field2d = new Field2d();
    SmartDashboard.putData(field2d);

  }

  public void driveTank(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public double getAngle() {
    return navX.getAngle();
  }

  public void stopDrive() {
    drive.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    field2d.setRobotPose(odometry.getPoseMeters());
    drive.feed();
  }
}
