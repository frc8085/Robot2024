package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants.Position;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class MoveToPositionAuto extends SequentialCommandGroup {
    public MoveToPositionAuto(
            ArmSubsystem m_arm,
            ShooterSubsystem m_shooter,
            FeederSubsystem m_feeder,
            Blinkin m_blinkin,
            Position position) {

        // Write to log what position we are moving to
        addCommands(
                new InstantCommand(() -> System.out.println("START Move to " + position.label + " Auto Position**")));
        // Check if we move Arm and Shooter Pivot at the same time
        if (position.parallelMovement) {
            addCommands(new InstantCommand(() -> m_arm.moveToPositionInParallel(position)));
        }

        // If they don't move together, check if we need to move the Arm first
        else if (position.moveArmFirst) {
            addCommands(
                    new InstantCommand(() -> m_arm.setArmPosition(position.armPosition)),
                    new WaitCommand(0.25),
                    new InstantCommand(() -> m_arm.setShooterPivotPosition(position.shooterPivotPosition,
                            position.shooterPivotAdjust)));
        }

        // Otherwise, move the Shooter Pivot first, then the Arm
        else {
            addCommands(
                    new InstantCommand(() -> m_arm.setShooterPivotPosition(position.shooterPivotPosition,
                            position.shooterPivotAdjust)),
                    new WaitCommand(.25),
                    new InstantCommand(() -> m_arm.setArmPosition(position.armPosition)));
        }

    }
}
