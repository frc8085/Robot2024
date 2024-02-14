package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.ArmConstants.Position;

public class MoveToPositionFromHome extends SequentialCommandGroup {
    public MoveToPositionFromHome(
            ArmSubsystem m_arm,
            IntakeSubsystem m_intake,
            ShooterSubsystem m_shooter,
            Position position) {
        addCommands(
                // new InstantCommand(() -> System.out.println("**START Move to " + position.label + " Position** ")),
                // new InstantCommand(() -> System.out.println(position.label + "ARM:" + position.armPosition)),
                // new InstantCommand(
                //         () -> System.out.println(position.label + "SHOOTER ARM:" + position.shooterArmPosition))
        new ParallelCommandGroup(
            new InstantCommand(() -> m_arm.moveToPosition(position)),
            new InstantCommand(() -> m_intake.runMove()))
        );
    }

}
