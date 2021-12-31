package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  CANSparkMax leftSpark;
  CANSparkMax rightSpark;

  RelativeEncoder leftEncoder;
  RelativeEncoder righEncoder;

  AHRS navX;
  SimDeviceSim navXSim;

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
    righEncoder = rightSpark.getEncoder();

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
      KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
      KitbotGearing.k10p71,        // 10.71:1
      KitbotWheelSize.kSixInch,     // 6" diameter wheels.
      null                         // No measurement noise.
      );
    // driveSim = new DifferentialDrivetrainSim( 
    //   DCMotor.getNEO(1),
    //   7.29,                    // 7.29:1 gearing reduction.
    //   7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
    //   60.0,                    // The mass of the robot is 60 kg.
    //   0.0508,                  // The robot uses 3" radius wheels.
    //   0.381 * 2,               // The track width is 0.7112 meters.
    //   null);
  }

  //Drive Tank
  public void driveTank(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  //Get angle from gyro
  public double getAngle() {
    return navX.getAngle();
  }

  //Stop all drive motors
  public void stopDrive() {
    drive.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(Rotation2d.fromDegrees(getAngle()), leftEncoder.getPosition(), righEncoder.getPosition());
    field2d.setRobotPose(odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    //NOTE get() is - as our motor controllers are inverted (inversion happens on output, not on set()/get())
    driveSim.setInputs(-leftSpark.get() * RobotController.getInputVoltage(),
    -rightSpark.get() * RobotController.getInputVoltage());

    driveSim.update(0.02);

    setSimDoubleFromDeviceData("navX-Sensor[0]", "Yaw", driveSim.getHeading().getDegrees());
    setSimDoubleFromDeviceData("SPARK MAX [4]", "Position", driveSim.getLeftPositionMeters());
    setSimDoubleFromDeviceData("SPARK MAX [5]", "Position", driveSim.getRightPositionMeters());
    setSimDoubleFromDeviceData("SPARK MAX [4]", "Applied Output", leftSpark.get());
    setSimDoubleFromDeviceData("SPARK MAX [5]", "Applied Output", rightSpark.get());
  }

  //Allows for shorter more direct references of SimDoubles
  public void setSimDoubleFromDeviceData(String deviceName, String doubleName, double value) {
    int device = SimDeviceDataJNI.getSimDeviceHandle(deviceName);
    SimDouble simDouble = new SimDouble(SimDeviceDataJNI.getSimValueHandle(device, doubleName));
    simDouble.set(value);
  }

}
