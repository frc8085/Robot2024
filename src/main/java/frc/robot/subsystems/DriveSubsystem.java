// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotDimensionConstants;

public class DriveSubsystem extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    public DriveSubsystem() {
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command drive(double xSpeed, double ySpeed, double rot) {

        return null;
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a
     * digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    private final Pigeon2 m_gyro = new Pigeon2(CanIDConstants.kGyroCanID);

    public Field2d field = new Field2d();

    // fien fein

    // Slew rate filter variables for controlling lateral acceleration
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    // // Odometry class for tracking robot pose
    // SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    // DriveConstants.kDriveKinematics,
    // getYaw(),
    // getModulePositions());

    SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            getYaw(),
            getModulePositions(),
            new Pose2d());

    double vWheel;

    public double calculateWheel(double omega, double VelocityX, double VelocityY, double heading) {
        // Omega: (-1) - 1
        // X/Y Velocity: 0 - 1
        double radius = Math
                .sqrt((Math.pow(RobotDimensionConstants.kLength, 2) + Math.pow(RobotDimensionConstants.kWidth, 2)));
        double RotationVelocity = omega * radius;

        double LinearAngle = Math.atan2(VelocityX, VelocityY);
        double vWheelAngle = (Math.atan2(RobotDimensionConstants.kWidth, RobotDimensionConstants.kLength) + 90);

        double LinearVelocity = Math.sqrt(Math.pow(VelocityX, 2) + Math.pow(VelocityY, 2));

        double LinearX = Math.cos(LinearAngle) * LinearVelocity;
        double LinearY = Math.sin(LinearAngle) * LinearVelocity;

        double RotationX = Math.cos(vWheelAngle) * RotationVelocity;
        double RotationY = Math.sin(vWheelAngle) * RotationVelocity;

        double vWheelSpeed = Math.sqrt(Math.pow((LinearX + RotationX), 2) + Math.pow((LinearY + RotationY), 2));

    }

}
