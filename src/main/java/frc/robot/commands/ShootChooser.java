package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants.ArmConstants.Position;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootChooser extends SequentialCommandGroup {
    public ShootChooser(
            FeederSubsystem m_feeder,
            ArmSubsystem m_arm,
            ShooterSubsystem m_shooter,
            Blinkin m_blinkin) {
        addCommands(
                new ConditionalCommand(
                        // If robot is in amp position, use ShootAmp
                        new ShootAmp(m_feeder, m_arm, m_shooter, m_blinkin, Position.AMP),
                        // If robot is in other position, use ShootNew
                        new ShootNew(m_feeder, m_arm, m_shooter, m_blinkin, Position.HOME),
                        m_arm::atAmpPosition));
    }

}