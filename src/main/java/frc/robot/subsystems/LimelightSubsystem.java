// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {
    /** Creates a new LimelightSubsystem. */
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    DriveSubsystem m_drive;

    public static HttpCamera m_limelight;

    private boolean m_visionMode;

    public LimelightSubsystem(
        DriveSubsystem drive
    ) {
        m_drive = drive;

        m_limelight = new HttpCamera("LL", "http://limelight:5809/stream.mjpg");
        m_limelight.setResolution(320, 240);
        m_limelight.setFPS(90);

        CameraServer.addCamera(m_limelight);

    }

    public static double getAprilTagID() {
        // The computer fought with us :( thats why it looks so weird
        double[] id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid")
                .getDoubleArray(new double[6]);
        return id[0];
    };

    public static double[] getRobotLocation() {
        // gets robot position realtive to AprilTag
        double[] location = NetworkTableInstance.getDefault().getTable("limelight")
                .getEntry("botpose_targetspace")
                .getDoubleArray(new double[6]);
        return location;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // read values periodically
        // SmartDashboard.putData(SendableCameraWrapper.wrap(m_limelight));
        // SmartDashboard.putData(SendableCameraWrapper.wrap(m_limelightRight));
        if (Constants.TuningModeConstants.kLimelightTuning) {

            SmartDashboard.putNumber("April tag ID", getAprilTagID());
            SmartDashboard.putNumberArray("LL Bot Location", getRobotLocation());
            SmartDashboard.putBoolean("Target", hasTarget());

            SmartDashboard.putNumber("LL ID", getID());

            SmartDashboard.putNumber("LL X", getX());
            SmartDashboard.putNumber("LL Y", getY());

        }

    }

    public double getX() {
        return table.getEntry("tx").getDouble(0.0);
    }

    // Account for the Robot's current heading when returning the X value
    public double getXfromRobotPerspective() {
        // Heading from -180 to 180
        double robotHeading = m_drive.getHeadingWrappedDegrees();
        double degreesToTarget = getX();
        double degreesFromRobotPerspective = robotHeading + degreesToTarget;
        return degreesFromRobotPerspective;
    }

    public double getY() {
        return table.getEntry("ty").getDouble(0.0);
    }

    public double getArea() {
        return table.getEntry("ta").getDouble(0.0);
    }

    public boolean hasTarget() {
        return table.getEntry("tv").getDouble(0.0) == 1;
    }

    public int getID() {
        return (int) table.getEntry("tid").getDouble(0.0);
    }

    public double getLatPip() {
        return table.getEntry("tl").getDouble(0.0) / 1000.0;
    }

    public double getLatCap() {
        return table.getEntry("cl").getDouble(0.0) / 1000.0;
    }

    public double getLastEntryTimeStamp() {
        return Timer.getFPGATimestamp() - getLatCap() - getLatPip();
    }

    /**
     * @param piplineNumber driver = 0, aprilTags = 1, retroreflective = 2
     */
    public void setPipeline(int pipelineNumber) {
        Number numObj = (Number) pipelineNumber;
        table.getEntry("pipeline").setNumber(numObj);
    }

    public boolean inVisionMode() {
        return m_visionMode;
    }

    public void setVisionModeOn() {
        m_visionMode = true;
    }

    public void setVisionModeOff() {
        m_visionMode = false;
    }
}
