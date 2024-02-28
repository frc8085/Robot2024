package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoYAim extends SequentialCommandGroup {
    public AutoYAim(
            ArmSubsystem m_arm,
            LimelightSubsystem m_limelight) {
        addCommands(
                new InstantCommand(() -> m_arm.setShooterPivotPosition(m_limelight.getYfromRobotPerspective())));
    }

}
