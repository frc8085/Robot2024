package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class MoveToPodium extends SequentialCommandGroup {
    public MoveToPodium(
            ArmSubsystem m_arm) {
        addCommands(
                // new ParallelCommandGroup(
                new InstantCommand(() ->
                m_arm.keepArmPosition(ArmConstants.podiumArmPosition)),
                new InstantCommand(() -> m_arm.keepShooterPivotPosition(ArmConstants.podiumShooterPivotPosition)));
    }

}
