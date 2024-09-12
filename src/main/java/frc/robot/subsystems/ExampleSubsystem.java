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

    public void relativeMotorRot(double heading, double d_Direction, double currentWheelSpeed, double d_Rotation, double d_RotationSensitivity) {
        // d_Direction is desired direction as an angle
        // rotation is desired rotation amount (max amount is 1, minimum is -1)
        // heading is direction robot is facing as an angle (field relative)

        double adjustedWheelRot = d_Direction - heading; // sets wheels to desired direction direction despite robot rotation
        
        double individualWheelSpeed = currentWheelSpeed -  
    }

    public double calculateRot(double d_Rotation, double LinearVelocity) {
        double omega = Math.toRadians(d_Rotation);

    }

}
