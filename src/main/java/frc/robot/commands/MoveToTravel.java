package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class MoveToTravel extends SequentialCommandGroup {
    public MoveToTravel(
            ArmSubsystem m_arm) {
        addCommands(
                // new ParallelCommandGroup(
                // new InstantCommand(() ->
                // m_arm.keepArmPosition(ArmConstants.travelArmPosition)),
                new InstantCommand(() -> m_arm.keepShooterPivotPosition(ArmConstants.travelShooterPivotPosition)));
    }

}
