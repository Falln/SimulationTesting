package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class ConstantsPW {
 
    //Drivetrain Feedforward and Feedback gains (constants) derived from Robot Characterization
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    public static final double kPDriveVel = 8.5;

    //Drivetrain kinematics derived from Robot Characterization. Allows conversion from chassisSpeed to wheelSpeed
    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    //Known maximums of the drivetrain derived from Robot Characterization
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    //Constants for the Ramsete Followers - in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}