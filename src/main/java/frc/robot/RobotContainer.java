// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants.Position;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.MoveToPosition;
import frc.robot.commands.NoteCheckAuto;
import frc.robot.commands.NoteCorrection;
import frc.robot.commands.PickUpNote;
import frc.robot.commands.PickUpNoteAuto;
import frc.robot.commands.PickUpNoteCompleted;
import frc.robot.commands.ShootChooser;
import frc.robot.commands.AutoTargetSP;
import frc.robot.commands.EjectNote;
import frc.robot.commands.EnableShooterAuto;
import frc.robot.commands.EnableShooterTrap;
import frc.robot.commands.ShootNew;
import frc.robot.commands.ShootTrap;
import frc.robot.commands.TargetSPTwice;
import frc.robot.commands.TargetTwice;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        private final SendableChooser<Command> autoChooser;
        protected SendableChooser<Alliance> allianceColor = new SendableChooser<>();

        private final Field2d field;

        // The robot's subsystems
        private final DriveSubsystem m_drive = new DriveSubsystem();
        public final ShooterSubsystem m_shooter = new ShooterSubsystem();
        private final IntakeSubsystem m_intake = new IntakeSubsystem();
        public final FeederSubsystem m_feeder = new FeederSubsystem();
        public final ArmSubsystem m_arm = new ArmSubsystem();
        private final ClimberSubsystem m_climb = new ClimberSubsystem();
        private final LimelightSubsystem m_limelight = new LimelightSubsystem(m_drive, m_arm);
        private final Blinkin m_blinkin = new Blinkin();

        // Register Named Commands for PathPlanner
        private void configureAutoCommands() {
                NamedCommands.registerCommand("TurnOnShooter", new ConditionalCommand(
                                new EnableShooterAuto(m_feeder, m_shooter),
                                new InstantCommand(),
                                m_feeder::noteInRobot));
                NamedCommands.registerCommand("MoveToSubwoofer", new ConditionalCommand(
                                new MoveToPosition(m_arm, m_shooter, m_feeder, m_blinkin, Position.AUTO_SUBWOOFER),
                                new InstantCommand(),
                                m_feeder::noteInRobot));
                NamedCommands.registerCommand("MoveToPodium", new ConditionalCommand(
                                new MoveToPosition(m_arm, m_shooter, m_feeder, m_blinkin, Position.PODIUM),
                                new InstantCommand(),
                                m_feeder::noteInRobot));
                NamedCommands.registerCommand("Shoot", new ConditionalCommand(
                                new ShootNew(m_feeder, m_arm, m_shooter, m_blinkin, Position.HOME),
                                new InstantCommand(),
                                m_feeder::noteInRobot));
                NamedCommands.registerCommand("PickUpNote",
                                new PickUpNoteAuto(m_intake, m_feeder, m_arm, m_shooter, m_blinkin));
                NamedCommands.registerCommand("PickUpNoteCompleted",
                                new PickUpNoteCompleted(m_intake, m_feeder, m_blinkin));
                NamedCommands.registerCommand("WaitUntilNoteDetected", new WaitUntilCommand(m_feeder::isNoteDetected));
                NamedCommands.registerCommand("WaitUntilHome", new WaitUntilCommand(m_arm::atHomePosition));
                NamedCommands.registerCommand("NoteCheckAuto", new NoteCheckAuto(m_intake, m_feeder));
                NamedCommands.registerCommand("NoteInRobot", new InstantCommand(m_feeder::notePickedUp));
                NamedCommands.registerCommand("WaitUntilReadyToShoot", new WaitUntilCommand(m_shooter::readyToShoot));

        }

        // The driver's controller
        CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
        // The operator's controller
        CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();
                addToDashboard();

                // Register Named Commands for Pathplanner
                configureAutoCommands();

                // Configure default commands
                m_drive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(() -> m_drive.drive(
                                                m_driverController.getRightTriggerAxis(),
                                                MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                                OIConstants.kDriveDeadband),
                                                MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                                OIConstants.kDriveDeadband),
                                                -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                                OIConstants.kDriveDeadband),
                                                true,
                                                true),
                                                m_drive));

                // Another option that allows you to specify the default auto by its name
                autoChooser = AutoBuilder.buildAutoChooser("Test Auto");

                SmartDashboard.putData("Auto Chooser", autoChooser);

                field = new Field2d();
                SmartDashboard.putData("Field", field);

                // Logging callback for current robot pose
                PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
                        // Do whatever you want with the pose here
                        field.setRobotPose(pose);
                });

                // Logging callback for target robot pose
                PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
                        // Do whatever you want with the pose here
                        field.getObject("target pose").setPose(pose);
                });

                // Logging callback for the active path, this is sent as a list of poses
                PathPlannerLogging.setLogActivePathCallback((poses) -> {
                        // Do whatever you want with the poses here
                        field.getObject("path").setPoses(poses);
                });

        }

        private void addToDashboard() {
                // Put a button on the dashboard for each setpoint
                // for (Position pos : Position.values()) {
                // SmartDashboard.putData(pos.label, new MoveToPosition(m_arm, m_shooter,
                // m_blinkin, pos));
                // }

                // SmartDashboard.putData("Trap Approach",
                // Commands.sequence(new MoveToPosition(m_arm, m_shooter, m_blinkin,
                // Position.TRAP_APPROACH)));
                // SmartDashboard.putData("Trap Climb",
                // Commands.sequence(
                // new MoveToPosition(m_arm, m_shooter, m_blinkin, Position.TRAP_FINAL)));
                // SmartDashboard.putData("Trap Score",
                // Commands.sequence(
                // new MoveToPosition(m_arm, m_shooter, m_blinkin, Position.TRAP_SCORE)));

                // // Intake Eject on dashboard
                SmartDashboard.putData("Eject", Commands.sequence(new InstantCommand(m_intake::eject),
                                new InstantCommand(m_feeder::eject)));

        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         */
        private void configureButtonBindings() {

                /*
                 * Driver Controls:
                 * Y Button:
                 * B Button:
                 * A Button:
                 * X Button:
                 * Start Button: Zero Heading
                 * DPad Left:
                 * DPad Up:
                 * DPad Right:
                 * DPad Down: Lock Wheels (set X)
                 * Left Bumper: Auto Target
                 * Right Bumper:
                 * Left Trigger: Shoot Note
                 * Right Trigger: Drive Speed
                 * Left Stick Button:
                 * Right Stick Button:
                 * 
                 * 
                 * Operator Controls:
                 * Y Button: Move Shooter to Home Position
                 * B Button: Move Shooter to Podium Position [If holding left bumper,to high
                 * podium]
                 * A Button: Move Shooter to Amp Position
                 * X Button: Move Shooter to Subwoofer Position [If holding left bumper, to
                 * Backwards Subwoofer]
                 * Start Button: Toggle Eject Note
                 * DPad Left: Move Shooter to Trap Approach Position
                 * DPad Up: Lower Robot from Climb
                 * DPad Right: Move Shooter to Trap Score Position
                 * DPad Down: Raise Robot in Climb
                 * Left Bumper: Alternate Position [Hold to Use]
                 * Right Bumper: Shoot Note into Trap during Climb
                 * Left Trigger: Toggle Intake
                 * Right Trigger: Toggle Shooter Wheels
                 * Left Stick Button: <No Op>
                 * Right Stick Button: Back Subwoofer
                 * *
                 */

                // DRIVER controlled buttons
                final Trigger shoot = m_driverController.leftTrigger();
                final Trigger lockWheels = m_driverController.povDown();

                final Trigger autoTarget = m_driverController.leftBumper();

                autoTarget.onTrue(new SequentialCommandGroup(
                                new TargetTwice(m_limelight, m_drive),
                                new TargetSPTwice(m_limelight, m_arm)));

                lockWheels.toggleOnTrue(new RunCommand(() -> m_drive.lock(),
                                m_drive));

                final Trigger zeroHeadingButton = m_driverController.start();

                zeroHeadingButton.onTrue(new InstantCommand(() -> m_drive.zeroHeading(), m_drive));

                // OPERATOR controlled buttons
                final Trigger toggleShooter = m_operatorController.rightTrigger();
                final Trigger toggleIntake = m_operatorController.leftTrigger();
                final Trigger ejectNote = m_operatorController.start();

                final Trigger alternatePosition = m_operatorController.leftBumper();
                final Trigger shootTrap = m_operatorController.rightBumper();

                final Trigger moveToHome = m_operatorController.y();
                final Trigger moveToSubwoofer = m_operatorController.x();
                final Trigger moveToAmp = m_operatorController.a();
                final Trigger moveToPodium = m_operatorController.b();
                final Trigger moveToBackSubwoofer = m_operatorController.rightStick();

                // Climb Controls TBD
                final Trigger moveToTrapApproach = m_operatorController.povLeft();
                final Trigger moveToTrapScore = m_operatorController.povRight();
                // final Trigger moveToTrapClimb = m_operatorController.povDown();

                // manual arm and shooter movement - arm left joystick, shooter right joystick
                final Trigger ArmRaiseButton = m_operatorController.axisLessThan(1, -.25);
                final Trigger ArmLowerButton = m_operatorController.axisGreaterThan(1, .25);
                final Trigger ShooterPivotLowerButton = m_operatorController.axisLessThan(5, -.25);
                final Trigger ShooterPivotRaiseButton = m_operatorController.axisGreaterThan(5, .25);

                final Trigger WinchForwardButton = m_operatorController.povDown();
                final Trigger WinchBackButton = m_operatorController.povUp();

                // Testing conditional, check if shooter is at speed, if it is, shoot, if not,
                // wait til it is at speed then shoot

                // Add another conditional command to shootnew to check if arm is in amp
                // position, so it doesn't return home after shooting
                shoot.onTrue(new ShootChooser(m_feeder, m_arm, m_shooter, m_blinkin));

                // intake.onTrue(new PickUpNote(m_intake, m_feeder, m_arm, m_shooter,
                // m_blinkin));

                ejectNote.toggleOnTrue(new ConditionalCommand(
                                new ParallelCommandGroup(
                                                new InstantCommand(m_intake::stop),
                                                new InstantCommand(m_feeder::stop),
                                                new InstantCommand(m_blinkin::driving)),
                                new EjectNote(m_intake, m_feeder, m_arm, m_shooter, m_blinkin),
                                m_intake::isIntakeRunning));

                toggleIntake.toggleOnTrue(new ConditionalCommand(
                                new ParallelCommandGroup(
                                                new InstantCommand(m_intake::stop),
                                                new InstantCommand(m_feeder::stop),
                                                new InstantCommand(m_blinkin::driving)),
                                new PickUpNote(m_intake, m_feeder, m_arm, m_shooter, m_blinkin),
                                m_intake::isIntakeRunning));

                // Operator Shooter Controls
                toggleShooter.toggleOnTrue(
                                new ConditionalCommand(new InstantCommand(m_shooter::stop),
                                                new ConditionalCommand(
                                                                new NoteCorrection(m_feeder)
                                                                                .andThen(new InstantCommand(
                                                                                                m_shooter::run)),
                                                                new InstantCommand(m_shooter::run),
                                                                m_feeder::needNoteCorrection),
                                                m_shooter::isShooterRunning));

                moveToHome.onTrue(
                                new ParallelCommandGroup(
                                                new MoveToPosition(m_arm, m_shooter, m_feeder, m_blinkin,
                                                                Position.HOME),
                                                new InstantCommand(m_feeder::stop),
                                                new InstantCommand(m_shooter::stop)));
                moveToSubwoofer.onTrue(
                                new MoveToPosition(m_arm, m_shooter, m_feeder, m_blinkin, Position.SUBWOOFER));
                moveToAmp.onTrue(new MoveToPosition(m_arm, m_shooter, m_feeder, m_blinkin, Position.AMP));
                moveToPodium.onTrue(new MoveToPosition(m_arm, m_shooter, m_feeder, m_blinkin, Position.PODIUM));

                /**
                 * Alternate positions. For these, you need to hold down the Left Bumper too.
                 **/
                // HIGH Podium
                moveToPodium.and(alternatePosition).onTrue(
                                new MoveToPosition(m_arm, m_shooter, m_feeder, m_blinkin, Position.HIGH_PODIUM));
                // // BACK Podium
                // moveToAmp.and(alternatePosition).onTrue(
                // new MoveToPosition(m_arm, m_shooter, m_blinkin, Position.BACK_PODIUM));
                // // HIGH Subwoofer
                moveToSubwoofer.and(alternatePosition).onTrue(
                                new MoveToPosition(m_arm, m_shooter, m_feeder, m_blinkin,
                                                Position.BACK_SUBWOOFER));
                moveToBackSubwoofer.onTrue(
                                new MoveToPosition(m_arm, m_shooter, m_feeder, m_blinkin, Position.BACK_SUBWOOFER));

                /**
                 * Manual Arm raise and lower
                 **/
                ArmRaiseButton.whileTrue(new InstantCommand(m_arm::armRaise, m_arm))
                                .onFalse(new InstantCommand(
                                                () -> m_arm.setArmPosition(
                                                                m_arm.getArmPosition())));

                ArmLowerButton.whileTrue(new InstantCommand(m_arm::armLower, m_arm))
                                .onFalse(new InstantCommand(
                                                () -> m_arm.setArmPosition(
                                                                m_arm.getArmPosition())));

                ShooterPivotRaiseButton.whileTrue(new InstantCommand(m_arm::shooterPivotRaise, m_arm))
                                .onFalse(new InstantCommand(
                                                () -> m_arm.setShooterPivotPosition(
                                                                m_arm.getShooterPivotPosition())));

                ShooterPivotLowerButton.whileTrue(new InstantCommand(m_arm::shooterPivotLower, m_arm))
                                .onFalse(new InstantCommand(
                                                () -> m_arm.setShooterPivotPosition(
                                                                m_arm.getShooterPivotPosition())));

                // trap
                moveToTrapApproach.onTrue(new MoveToPosition(m_arm, m_shooter,
                                m_feeder, m_blinkin, Position.TRAP_APPROACH));
                moveToTrapScore.onTrue(new SequentialCommandGroup(
                                new MoveToPosition(m_arm, m_shooter, m_feeder, m_blinkin, Position.TRAP_SCORE),
                                new InstantCommand(m_shooter::runTrap)));
                shootTrap.onTrue(new ShootTrap(m_feeder, m_arm, m_shooter, m_blinkin,
                                Position.TRAP_FINAL));

                // Climber motor on and off
                WinchForwardButton.whileTrue(
                                new InstantCommand(m_climb::forward))
                                .onFalse(new InstantCommand(m_climb::stop));

                WinchBackButton.whileTrue(
                                new InstantCommand(m_climb::back))
                                .onFalse(new InstantCommand(m_climb::stop));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();

        }
}
