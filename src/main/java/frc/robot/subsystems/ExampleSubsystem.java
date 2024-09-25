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

    double vWheel;

    public double calculateWheel(double omega, double LinearVelocity, double heading, double d_Direction,
            double distance) {
        // Omega: (-1) - 1
        // LinearVelocity: 0 - 1
        double RotationVelocity = omega * distance;

        double vWheelAngle; // Unfinished

        double LinearX = Math.cos(vWheelAngle) * LinearVelocity;
        double LinearY = Math.sin(vWheelAngle) * LinearVelocity;

        double RotationX = Math.cos(vWheelAngle) * RotationVelocity;
        double RotationY = Math.sin(vWheelAngle) * RotationVelocity;

        double vWheelSpeed = Math.sqrt(Math.pow((LinearX + RotationX), 2) + Math.pow((LinearY + RotationY), 2));

    }

}
