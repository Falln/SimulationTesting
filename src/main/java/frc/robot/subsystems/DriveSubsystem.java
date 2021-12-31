package frc.robot.subsystems;

import javax.naming.directory.DirContext;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimValue;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  CANSparkMax leftSpark;
  CANSparkMax rightSpark;

  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;

  AHRS navX;
  SimDeviceSim navXSim;

  EncoderSim encoderSim;

  Field2d field2d;
  DifferentialDriveOdometry odometry;

  DifferentialDrive drive;
  DifferentialDrivetrainSim driveSim;


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    //Initalize different components
    leftSpark = new CANSparkMax(4, MotorType.kBrushless);
    rightSpark = new CANSparkMax(5, MotorType.kBrushless);
    leftSpark.setInverted(true);
    rightSpark.setInverted(true);

    leftEncoder = leftSpark.getEncoder();
    rightEncoder = rightSpark.getEncoder();

    navX = new AHRS(SPI.Port.kMXP);
    navXSim = new SimDeviceSim("navX-Sensor[0]");
    navX.reset();

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()));
    field2d = new Field2d();
    SmartDashboard.putData("Field", field2d);

    drive = new DifferentialDrive(leftSpark, rightSpark);
    drive.setDeadband(0.09); //Same thing as setting a min input value but a little different

    //Things needed for drive simulation
    driveSim = DifferentialDrivetrainSim.createKitbotSim(
      KitbotMotor.kDoubleNEOPerSide, // 2 CIMs per side.
      KitbotGearing.k10p71,        // 10.71:1
      KitbotWheelSize.kSixInch,     // 6" diameter wheels.
      null                         // No measurement noise.
      );
  }

  /** Set the power of the left and right sides of the drivetrain independently */
  public void driveTank(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  /** Get angle from gyro */
  public double getAngle() {
    return navX.getAngle();
  }

  /** Stop all drive motors */
  public void stopDrive() {
    drive.stopMotor();
  }

  /** Updates the drive odometry */
  public void updateOdometry() {
    odometry.update(Rotation2d.fromDegrees(getAngle()), leftEncoder.getPosition(), rightEncoder.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    field2d.setRobotPose(odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    //NOTE get() needs to be - as our motor controllers are inverted (inversion happens on output, not on set()/get())
    driveSim.setInputs(-leftSpark.get() * RobotController.getInputVoltage(), //12*.5 =6v
    -rightSpark.get() * RobotController.getInputVoltage());

    driveSim.update(0.02);

    //update navX and Spark data (as much as needed)
    setSimDoubleFromDeviceData("navX-Sensor[0]", "Yaw", driveSim.getHeading().getDegrees());
    setSimDoubleFromDeviceData("SPARK MAX [4]", "Position", driveSim.getLeftPositionMeters());
    setSimDoubleFromDeviceData("SPARK MAX [5]", "Position", driveSim.getRightPositionMeters());
    setSimDoubleFromDeviceData("SPARK MAX [4]", "Applied Output", leftSpark.get());
    setSimDoubleFromDeviceData("SPARK MAX [5]", "Applied Output", rightSpark.get());
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
