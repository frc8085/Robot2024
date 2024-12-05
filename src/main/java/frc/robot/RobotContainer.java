// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem m_drive = new DriveSubsystem();

    // private final CommandXboxController m_driverController = new
    // CommandXboxController(OperatorConstants.kDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.

    // EXAMPLE BINDINGS
    private void configureButtonBindings() {
        // DRIVER controlled buttons
        // final Trigger shoot = m_drivercontroller.leftTrigger();
        // final Trigger shootInstant = m_driverController.rightBumper();
    }
}