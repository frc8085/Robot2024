package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants.Position;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.ShooterSubsystem;

public class MoveToPosition extends SequentialCommandGroup {
    public MoveToPosition(
            ArmSubsystem m_arm,
            ShooterSubsystem m_shooter,
            Blinkin m_blinkin,
            Position position) {

        // Check if the arm is at a height where if shooter goes vertical the robot will
        // be > 48 inches
        // if (m_arm.armShooterAboveMaxHeight()) {
        // addCommands(new InstantCommand(() -> m_arm.setArmPosition(200)),
        // new WaitUntilCommand(m_arm::armShooterBelowMaxHeight));
        // }
        // Check if we move Arm and Shooter Pivot at the same time
        if (position.parallelMovement) {
            addCommands(new InstantCommand(() -> m_arm.moveToPosition(position)));
        }

        // If they don't move together, check if we need to move the Arm first
        else if (position.moveArmFirst) {
            addCommands(
                    new InstantCommand(() -> m_arm.setArmPosition(position.armPosition)),
                    new WaitCommand(0.5),
                    new InstantCommand(() -> m_arm.setShooterPivotPosition(position.shooterPivotPosition)));
        }

        // Otherwise, move the Shooter Pivot first, then the Arm
        else {
            addCommands(
                    new InstantCommand(() -> m_arm.setShooterPivotPosition(position.shooterPivotPosition)),
                    new WaitCommand(0.5),
                    new InstantCommand(() -> m_arm.setArmPosition(position.armPosition)));
        }

        if (position.shooterOn) {
            addCommands(
                    new InstantCommand(m_shooter::run),
                    new WaitUntilCommand(m_shooter::readyToShoot),
                    new InstantCommand(m_blinkin::shooterAtSetPoint));
        } else {
            addCommands(
                    new InstantCommand(m_shooter::stop));

        }
    }
}
