package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public final class Autos {

    public static Command TheAuto(
            DriveSubsystem m_drive,
            IntakeSubsystem m_intake,
            ArmSubsystem m_arm,
            FeederSubsystem m_feeder,
            ShooterSubsystem m_shooter,
            Blinkin m_blinkin) {

        return Commands.sequence(
                // 1. AimAndShoot1()
                new AutoAimAndShoot(m_arm, m_shooter, ArmConstants.Position.SUBWOOFER),
                // 2. Drive(FirstPickup)
                // 3. Pickup
                new PickUpNote(m_intake, m_feeder, m_arm, m_shooter, m_blinkin),
                // 4. Drive(Diagonal)
                // 5. AimAndShoot2()
                // 6. Drive(Pickup)
                // 7. Pickup
                new PickUpNote(m_intake, m_feeder, m_arm, m_shooter, m_blinkin),
                // 8. Drive(Diagonal)
                // 9. AimAndShoot3()
                // 10. Drive(Pickup)
                // 11. Pickup
                new PickUpNote(m_intake, m_feeder, m_arm, m_shooter, m_blinkin)
        // 12. AimAndShoot4()
        );
    }
}
