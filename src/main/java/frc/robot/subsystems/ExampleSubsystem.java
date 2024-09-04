// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    public ExampleSubsystem() {
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

    public void relativeMotorRot(double d_Direction, double rotation, double rotationSensitivity, double heading) {
        // d_Direction is desired direction as an angle
        // rotation is desired rotation amount (max amount is 1, minimum is -1)
        // heading is direction robot is facing as an angle
        double FLMax;
        double FRMax;
        double BRMax;
        double BLMax;

        // Max rotation angles
        if (rotation > 0) {
            FLMax = 70;
            FRMax = 140;
            BRMax = 210;
            BLMax = 280;
        } else {
            FLMax = 140;
            FRMax = 70;
            BRMax = 280;
            BLMax = 210;
        }

        double wheelOffset = d_Direction - heading;

        double FLRot = (wheelOffset + calculateRot(FLMax, rotation, rotationSensitivity)) / 2;
        double FRRot = (wheelOffset + calculateRot(FRMax, rotation, rotationSensitivity)) / 2;
        double BLRot = (wheelOffset + calculateRot(BLMax, rotation, rotationSensitivity)) / 2;
        double BRRot = (wheelOffset + calculateRot(BRMax, rotation, rotationSensitivity)) / 2;

        // btw this code wont work probably
    }

    public double calculateRot(double x, double y, double z) {
        // x: maxrot
        // y: rotation
        // z: rotationSensitivity
        double rotAngle = x * y * z;
        return rotAngle;
    }

}
