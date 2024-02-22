// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightSubsystem {
    private boolean TUNING_MODE = false;
    // private double _hearBeatPeriod = 0.1;

    public void log() {
        // Things to show only in tuninig mode
        if (TUNING_MODE) {
            SmartDashboard.putNumberArray("Distance to Target", getDistanceToTarget());
            SmartDashboard.putNumber("Rotation to Target", getdegRotationToTarget());
            // SmartDashboard.putNumber("Target Setpoint", getSetpointToTarget());
            SmartDashboard.putNumber("vertical distance to target", getdegVerticalToTarget());
            SmartDashboard.putNumber("Apriltag ID", getAprilTagID());
        }
    }

    public static double getAprilTagID() {
        double[] id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid")
                .getDoubleArray(new double[6]);
        return id[0];
    }

    public static double[] getDistanceToTarget() {
        double[] location = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace")
                .getDoubleArray(new double[6]);
        return location;
    }

    public static double getdegRotationToTarget() {
        NetworkTableEntry tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
        double x = tx.getDouble(0.0);
        return x;
    }

    public double getdegVerticalToTarget() {
        NetworkTableEntry ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
        double y = ty.getDouble(0.0);
        return y;
    }
}

// * NOTE:
// * What neural network model should we use? Teachable machine is a classifer
// * meaning it classifies stuff.
// * However a detection model will require a bit more work and *some* python (I
// * can do that)
// * but, as in the name, it only detects which is what we need it for. -Frank

// >:)