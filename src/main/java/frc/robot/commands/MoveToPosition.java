package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem;

import static frc.robot.Constants.ArmConstants.Position;

public class MoveToPosition extends SequentialCommandGroup {
    public MoveToPosition(
            ArmSubsystem m_arm,
            Position position) {

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
    }
}
