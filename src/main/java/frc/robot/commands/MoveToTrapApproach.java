package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class MoveToTrapApproach extends SequentialCommandGroup {
    public MoveToTrapApproach(
            ArmSubsystem m_arm) {
        addCommands(
                new InstantCommand(() -> System.out.println("**START Move to Travel Position**")),
                new ParallelCommandGroup(
                    new InstantCommand(() -> m_arm.setArmPositionDegrees(ArmConstants.kTrapApproachArm)),
                    new InstantCommand(() -> m_arm.setShooterArmPositionDegrees(ArmConstants.kTrapApproachShooter))
                    )
        );
    }
}
