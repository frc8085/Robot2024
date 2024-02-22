// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
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
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants.Position;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.PickUpNote;
import frc.robot.commands.PickUpNoteCompleted;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootTrap;
import frc.robot.commands.MoveToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.FeederSubsystem;
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
        private final ShooterSubsystem m_shooter = new ShooterSubsystem();
        private final IntakeSubsystem m_intake = new IntakeSubsystem();
        private final FeederSubsystem m_feeder = new FeederSubsystem();
        private final ArmSubsystem m_arm = new ArmSubsystem();
        private final ClimberSubsystem m_climb = new ClimberSubsystem();
        private final LimelightSubsystem m_limelight = new LimelightSubsystem();
        private final Blinkin m_blinkin = new Blinkin();

        // Register Named Commands for PathPlanner
        private void configureAutoCommands() {
                NamedCommands.registerCommand("TurnOnShooter", new InstantCommand(m_shooter::run));
                NamedCommands.registerCommand("MoveToSubwoofer",
                                new MoveToPosition(m_arm, m_shooter, m_blinkin, Position.LOW_SUBWOOFER));
                NamedCommands.registerCommand("MoveToPodium",
                                new MoveToPosition(m_arm, m_shooter, m_blinkin, Position.PODIUM));
                NamedCommands.registerCommand("Shoot", new Shoot(m_feeder, m_arm, m_shooter, m_blinkin, Position.HOME));
                NamedCommands.registerCommand("PickUpNote",
                                new PickUpNote(m_intake, m_feeder, m_arm, m_shooter, m_blinkin));
                NamedCommands.registerCommand("PickUpNoteDetected",
                                new PickUpNoteCompleted(m_intake, m_feeder, m_blinkin));
                NamedCommands.registerCommand("WaitUntilNoteDetected", new WaitUntilCommand(m_feeder::isNoteDetected));
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
                // SmartDashboard.putData(pos.label, new MoveToPosition(m_arm, m_shooter, pos));
                // }

                SmartDashboard.putData("Trap Approach",
                                Commands.sequence(new MoveToPosition(m_arm, m_shooter, m_blinkin,
                                                Position.TRAP_APPROACH)));
                SmartDashboard.putData("Trap Climb",
                                Commands.sequence(
                                                new MoveToPosition(m_arm, m_shooter, m_blinkin, Position.TRAP_CLIMB)));
                SmartDashboard.putData("Trap Score",
                                Commands.sequence(
                                                new MoveToPosition(m_arm, m_shooter, m_blinkin, Position.TRAP_SCORE)));

                // Intake Eject on dashboard
                SmartDashboard.putData("Eject", Commands.sequence(new InstantCommand(m_intake::eject),
                                new InstantCommand(m_feeder::eject)));

                // Stop Feeder and Intake
                SmartDashboard.putData("STOP intake", Commands.sequence(new InstantCommand(m_intake::stop),
                                new InstantCommand(m_feeder::stop)));

        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         */
        private void configureButtonBindings() {
                // DRIVER controlled buttons
                final Trigger shoot = m_driverController.leftTrigger();
                final Trigger turnOnShooter = m_driverController.x();
                final Trigger turnOffShooter = m_driverController.b();
                final Trigger lockWheels = m_driverController.povDown();
                final Trigger fieldRelative = m_driverController.leftBumper();
                final Trigger robotRelative = m_driverController.rightBumper();

                lockWheels.toggleOnTrue(new RunCommand(() -> m_drive.lock(),
                                m_drive));

                final Trigger zeroHeadingButton = m_driverController.start();

                zeroHeadingButton.onTrue(new InstantCommand(() -> m_drive.zeroHeading(), m_drive));

                // OPERATOR controlled buttons
                final Trigger toggleShooter = m_operatorController.rightTrigger();
                final Trigger intake = m_operatorController.leftTrigger();
                final Trigger stopIntake = m_operatorController.start();

                // eventually the operator will control the shooter wheels
                // final Trigger turnOnShooter = m_operatorController.rightTrigger();

                final Trigger alternatePosition = m_operatorController.leftBumper();
                final Trigger testLEDColors = m_operatorController.rightBumper();

                final Trigger moveToHome = m_operatorController.y();
                final Trigger moveToSubwoofer = m_operatorController.x();
                final Trigger moveToAmp = m_operatorController.a();
                final Trigger moveToPodium = m_operatorController.b();

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
                shoot.onTrue(new ConditionalCommand(new Shoot(m_feeder, m_arm, m_shooter, m_blinkin, Position.HOME),
                                new WaitUntilCommand(m_shooter::readyToShoot)
                                                .andThen(new Shoot(m_feeder, m_arm, m_shooter, m_blinkin,
                                                                Position.HOME)),
                                m_shooter::readyToShoot));

                intake.onTrue(new PickUpNote(m_intake, m_feeder, m_arm, m_shooter, m_blinkin))
                                .onFalse(new SequentialCommandGroup(
                                                new WaitUntilCommand(m_feeder::isNoteDetected),
                                                new PickUpNoteCompleted(m_intake, m_feeder, m_blinkin)));

                stopIntake.onTrue(new ParallelCommandGroup(new InstantCommand(m_intake::stop),
                                new InstantCommand(m_feeder::stop)));

                // Driver shooter controls
                turnOnShooter.onTrue(new InstantCommand(m_shooter::runTrap));
                turnOffShooter.onTrue(new InstantCommand(m_shooter::stop));

                // Operator Shooter Controls
                toggleShooter.toggleOnTrue(Commands.startEnd(m_shooter::run,
                                m_shooter::stop, m_shooter));
                toggleShooter.and(alternatePosition)
                                .toggleOnTrue(Commands.startEnd(m_shooter::runTrap, m_shooter::stop, m_shooter));

                moveToHome.onTrue(new MoveToPosition(m_arm, m_shooter, m_blinkin, Position.HOME));
                moveToSubwoofer.onTrue(
                                new MoveToPosition(m_arm, m_shooter, m_blinkin, Position.SUBWOOFER));
                moveToAmp.onTrue(new MoveToPosition(m_arm, m_shooter, m_blinkin, Position.AMP));
                moveToPodium.onTrue(new MoveToPosition(m_arm, m_shooter, m_blinkin, Position.PODIUM));

                /**
                 * Alternate positions. For these, you need to hold down the Left Bumper too.
                 **/
                // HIGH Podium
                // moveToPodium.and(alternatePosition).onTrue(
                // new MoveToPosition(m_arm, m_shooter, m_blinkin, Position.HIGH_PODIUM));
                // // BACK Podium
                // moveToAmp.and(alternatePosition).onTrue(
                // new MoveToPosition(m_arm, m_shooter, m_blinkin, Position.BACK_PODIUM));
                // // HIGH Subwoofer
                moveToSubwoofer.and(alternatePosition)
                                .onTrue(new MoveToPosition(m_arm, m_shooter, m_blinkin,
                                                Position.BACK_SUBWOOFER));

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
                                m_blinkin, Position.TRAP_APPROACH));
                moveToTrapScore.onTrue(new MoveToPosition(m_arm, m_shooter,
                                m_blinkin, Position.TRAP_SCORE));
                moveToAmp.and(alternatePosition).onTrue(new ShootTrap(m_feeder, m_arm, m_shooter, m_blinkin));

                // Climber motor on and off
                WinchForwardButton.whileTrue(
                                new InstantCommand(m_climb::forward))
                                .onFalse(new InstantCommand(m_climb::stop));

                WinchBackButton.whileTrue(
                                new InstantCommand(m_climb::back))
                                .onFalse(new InstantCommand(m_climb::stop));

                // Testing LED colors
                new ConditionalCommand(new InstantCommand(m_blinkin::withNote),
                                new InstantCommand(m_blinkin::shooterAtSetPoint), testLEDColors);

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

/**
 * Stuff Taken Out of the Autonomous command to try PathPlanner stuff
 * // Create config for trajectory
 * TrajectoryConfig config = new TrajectoryConfig(
 * AutoConstants.kMaxSpeedMetersPerSecond,
 * AutoConstants.kMaxAccelerationMetersPerSecondSquared)
 * // Add kinematics to ensure max speed is actually obeyed
 * .setKinematics(DriveConstants.kDriveKinematics);
 * 
 * // An example trajectory to follow. All units in meters.
 * Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
 * // Start at the origin facing the +X direction
 * new Pose2d(0, 0, new Rotation2d(0)),
 * // Pass through these two interior waypoints, making an 's' curve path
 * List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
 * // End 3 meters straight ahead of where we started, facing forward
 * new Pose2d(3, 0, new Rotation2d(0)),
 * config);
 * 
 * var thetaController = new ProfiledPIDController(
 * AutoConstants.kPThetaController, 0, 0,
 * AutoConstants.kThetaControllerConstraints);
 * thetaController.enableContinuousInput(-Math.PI, Math.PI);
 * 
 * SwerveControllerCommand swerveControllerCommand = new
 * SwerveControllerCommand(
 * exampleTrajectory,
 * m_drive::getPose, // Functional interface to feed supplier
 * DriveConstants.kDriveKinematics,
 * 
 * // Position controllers
 * new PIDController(AutoConstants.kPXController, 0, 0),
 * new PIDController(AutoConstants.kPYController, 0, 0),
 * thetaController,
 * m_drive::setModuleStates,
 * m_drive);
 * 
 * // Reset odometry to the starting pose of the trajectory.
 * m_drive.resetOdometry(exampleTrajectory.getInitialPose());
 * 
 * // Run path following command, then stop at the end.
 * return swerveControllerCommand.andThen(() -> m_drive.drive(0, 0, 0, 0, false,
 * false));
 */