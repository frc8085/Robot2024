// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

        public static final class CanIdConstants {
                public static final int kGyroCanId = 15;
                public static final int kIntakeCanId = 21;
                public static final int kWinchCanId = 22;
                public static final int kArmCanId = 23;
                public static final int kShooterPivotCanId = 24;
                public static final int kFeederCanId = 25;
                public static final int kShooter1CanId = 26;
                public static final int kShooter2CanId = 27;
        }

        public static final class LoggingConstants {
                public static final boolean kLogging = true;
        }

        public static final class MotorDefaultsConstants {
                public static final int NeoCurrentLimit = 40;
                public static final int NeoVortexCurrentLimit = 60;
                public static final int Neo550CurrentLimit = 20;
                public static final MotorType NeoMotorType = MotorType.kBrushless;
                public static final MotorType Neo550MotorType = MotorType.kBrushless;
                public static final MotorType NeoVortexMotorType = MotorType.kBrushless;
        }

        public static final class TuningModeConstants {
                public static final boolean kPracticeMode = false;
                public static final boolean kTuning = false;
                public static final boolean kDriveTuning = false;
                public static final boolean kArmTuning = false;
                public static final boolean kClimberTuning = false;
                public static final boolean kFeederTuning = false;
                public static final boolean kIntakeTuning = false;
                public static final boolean kLimelightTuning = false;
                public static final boolean kShooterTuning = false;
        }

        public static final class IntakeConstants {
                public static final double speed = .9;
                public static final double armMoveSpeed = .25;
                public static final double ejectSpeed = -1;
        }

        public static final class FeederConstants {
                public static final int kIRPort1 = 0;
                public static final int kIRPort2 = 1;

                public static final double speed = 1;
                public static final double pickupSpeed = .3;
                public static final double speedAuto = 1;
                public static final double kLoadWaitTime = 0;

                public static double kFeederMaxOutput = 1;
                public static double kFeederMinOutput = -1;
                public static double kFeederFF = 0.0001;
                public static double kFeederP = 0.0001;
                public static double kFeederI = 0.0001;
                public static double kFeederD = 0;

                public static double kFeederSetPoint = 4000;

        }

        public static final class ShooterConstants {

                public static final IdleMode kShooterMotor2IdleMode = IdleMode.kBrake;
                public static IdleMode kShooterMotor1IdleMode = IdleMode.kBrake;
                public static double kShooter2MaxOutput = 1;
                public static double kShooter2MinOutput = -1;
                public static double kShooter2FF = 0.000235;
                public static double kShooter2P = 0.0006;
                public static double kShooter2I = 0.00;
                public static double kShooter2D = .6;

                public static double kShooter1MaxOutput = 1;
                public static double kShooter1MinOutput = -1;
                public static double kShooter1FF = 0.000235;
                public static double kShooter1P = 0.0006;
                public static double kShooter1I = 0.00;
                public static double kShooter1D = .6;

                public static double kShooterEncoder2VelocityFactor = (2 * Math.PI) / 60.0;
                public static double kShooterEncoder2PositionFactor = (2 * Math.PI);
                public static double kShooterEncoder1VelocityFactor = (2 * Math.PI) / 60.0;
                public static double kShooterEncoder1PositionFactor = (2 * Math.PI);

                public static double kShooter1SetPoint = 4300;
                public static double kShooter2SetPoint = kShooter1SetPoint * 0.9;
                public static double kShooter1PodiumToleranceRPMPercent = .03;
                public static double kShooter2PodiumToleranceRPMPercent = .04;
                public static double kShooter1SWToleranceRPMPercent = .03;
                public static double kShooter2SWToleranceRPMPercent = .04;

                public static double kShooterSetPointTrap = 1000;
                public static double kShooterSetPointAmp = 1000;
                public static double kShooterOff = 0;
        }

        public static final class ArmConstants {

                public static final IdleMode kArmMotorIdleMode = IdleMode.kBrake;
                public static final IdleMode kShooterPivotMotorIdleMode = IdleMode.kBrake;

                public static final double kArmMaxSpeed = .75;
                public static final double kArmMinSpeed = -.75;
                public static final double kShooterPivotMaxSpeed = 1;

                // Manual Arm movement speeds
                public static final double kArmRaiseSpeed = .25;
                public static final double kArmLowerSpeed = .15;

                public static final double kShooterPivotRaiseSpeed = .25;
                public static final double kShooterPivotLowerSpeed = .25;

                // arm height where shooter is above max height
                public static final double shooterMaxHeight = 320;

                // shooter pivot minimum and maximum
                public static final double kShooterPivotMin = 20;
                public static final double kShooterPivotMax = 235;

                // Manual Adjustment degree differential
                public static final double kArmManualAdjustment = 0;
                public static final double kShooterPivotManualAdjustment = 5;

                // PIDS
                // Arm PID coefficients
                public static final int kArmPIDSlot = 0;
                public static final double kArmP = 0.020;
                public static final double kArmI = 0;
                public static final double kArmD = 0.1;
                public static final double kArmFF = 0.0012;
                public static final double kArmMaxOutput = .75;
                public static final double kArmMinOutput = -.7;
                public static final double kArmMaxAccel = .00000000006;
                public static final double kArmMaxVelo = .000000000014;

                // Shooter Pivot PID coefficients
                public static final int kShooterPivotPIDSlot = 0;
                public static final double kShooterPivotP = 0.01;
                public static final double kShooterPivotI = 0;
                public static final double kShooterPivotD = 0;
                public static final double kShooterPivotFF = 0.0005;
                public static final double kShooterPivotMaxOutput = kShooterPivotMaxSpeed;
                public static final double kShooterPivotMinOutput = -kShooterPivotMaxSpeed;
                public static final double kShooterPivotMaxAccel = .25;
                public static final double kShooterPivotMaxVelo = .5;

                // Arm PID coefficients
                public static final int kArmPIDSlot1 = 1;
                public static final double kArmPSlot1 = 0.017;
                public static final double kArmISlot1 = 0;
                public static final double kArmDSlot1 = 0.1;
                public static final double kArmFFSlot1 = 0.0012;
                public static final double kArmMaxOutputSlot1 = kArmMaxSpeed;
                public static final double kArmMinOutputSlot1 = kArmMinSpeed;
                public static final double kArmMaxAccelSlot1 = 24;
                public static final double kArmMaxVeloSlot1 = 14;

                // Shooter Pivot PID coefficients
                public static final int kShooterPivotPIDSlot1 = 1;
                public static final double kShooterPivotPSlot1 = 0.02;
                public static final double kShooterPivotISlot1 = 0;
                public static final double kShooterPivotDSlot1 = 0;
                public static final double kShooterPivotFFSlot1 = 0.0005;
                public static final double kShooterPivotMaxOutputSlot1 = kShooterPivotMaxSpeed;
                public static final double kShooterPivotMinOutputSlot1 = -kShooterPivotMaxSpeed;
                public static final double kShooterPivotMaxAccelSlot1 = .25;
                public static final double kShooterPivotMaxVeloSlot1 = .5;

                // SETPOINTS

                // Offset
                public static final double kArmZeroOffsetFactor = 147;
                public static final double kShooterPivotZeroOffsetFactor = 306;

                // We are zeroing the arm at the top limit, so all our positions should be
                // adjusted
                public static final double kArmPositionShift = 232;

                public static final double kArmAdjustmentFactor = 90 + kArmPositionShift;

                // the arm position is about 25 higher when it holds than what you want
                public static final double kArmPIDShift = 20;

                // Zeroing the shooter pivot at the far limit, so all our positions should be
                // adjusted
                public static final double kShooterPivotPositionShift = 47;

                public static final double kShooterPivotAdjustmentFactor = 0 + kShooterPivotPositionShift;

                // Arm Positions - Shooter Pivot lower number aims higher
                public enum Position {
                        HOME("Home",
                                        238,
                                        41,
                                        false,
                                        false,
                                        true,
                                        ShooterConstants.kShooterOff),
                        AMP("Amp",
                                        325,
                                        200,
                                        true,
                                        false,
                                        true,
                                        ShooterConstants.kShooter1SetPoint),
                        PODIUM("Podium",
                                        260,
                                        55,
                                        true,
                                        true,
                                        false,
                                        ShooterConstants.kShooter1SetPoint),
                        AUTO_PODIUM("Auto Podium",
                                        260,
                                        55,
                                        true,
                                        true,
                                        false,
                                        ShooterConstants.kShooter1SetPoint),
                        AUTO_PODIUM_SOURCE("Auto Podium Source",
                                        260,
                                        56,
                                        true,
                                        true,
                                        false,
                                        ShooterConstants.kShooter1SetPoint),
                        AUTO_PODIUM_AMP("Auto Podium Amp",
                                        260,
                                        55,
                                        true,
                                        true,
                                        false,
                                        ShooterConstants.kShooter1SetPoint),
                        SUBWOOFER("Subwoofer",
                                        325,
                                        116,
                                        true,
                                        false,
                                        true,
                                        ShooterConstants.kShooter1SetPoint),
                        AUTO_SUBWOOFER("Auto Subwoofer",
                                        285,
                                        65,
                                        true,
                                        true,
                                        true,
                                        ShooterConstants.kShooter1SetPoint),
                        SIDE_SUBWOOFER("Side Subwoofer",
                                        330,
                                        120,
                                        true,
                                        false,
                                        true,
                                        ShooterConstants.kShooter1SetPoint),

                        TRAP_APPROACH("Trap Approach",
                                        330,
                                        155,
                                        true,
                                        false,
                                        false,
                                        ShooterConstants.kShooterOff),
                        TRAP_SECOND("Trap Second",
                                        310,
                                        135,
                                        true,
                                        true,
                                        false,
                                        ShooterConstants.kShooterOff),

                        TRAP_FINAL("Trap Final",
                                        310,
                                        40,
                                        true,
                                        true,
                                        false,
                                        ShooterConstants.kShooterOff),
                        TRAP_SCORE("Trap Score",
                                        315,
                                        93,
                                        true,
                                        true,
                                        false,
                                        ShooterConstants.kShooterSetPointTrap),
                        HIGH_PODIUM("High Podium",
                                        322,
                                        135,
                                        true,
                                        false,
                                        true,
                                        ShooterConstants.kShooter1SetPoint),

                        BACK_PODIUM("Back Podium",
                                        338,
                                        161,
                                        true,
                                        true,
                                        true,
                                        ShooterConstants.kShooter1SetPoint),

                        BACK_SUBWOOFER("Back Subwoofer",
                                        325,
                                        37,
                                        true,
                                        true,
                                        true,
                                        ShooterConstants.kShooter1SetPoint),

                        EJECT_NOTE("Eject Note",
                                        280,
                                        41,
                                        true,
                                        true,
                                        false,
                                        ShooterConstants.kShooterOff);

                        public final String label;
                        public final double armPosition;
                        public final double shooterPivotPosition;
                        public final boolean moveArmFirst;
                        public final boolean parallelMovement;
                        public final boolean HeightCheck;
                        public final double shooterSpeed;

                        private Position(String label,
                                        double armPosition,
                                        double shooterPivotPosition,
                                        boolean moveArmFirst,
                                        boolean parallelMovement,
                                        boolean HeightCheck,
                                        double shooterSpeed) {
                                this.label = label;
                                this.armPosition = armPosition;
                                this.shooterPivotPosition = shooterPivotPosition;
                                this.moveArmFirst = moveArmFirst;
                                this.parallelMovement = parallelMovement;
                                this.HeightCheck = HeightCheck;
                                this.shooterSpeed = shooterSpeed;
                        }

                }

                // Estimates, fix this once we get exact measurements
                public static final double kArmTotalDegrees = 360;
                public static final double kArmTotalRevolutions = 1;

                // Convert angle of travel to encoder rotations, where encoder reading of .1 is
                // 0 degrees and reading of 5.5 is 90 degrees
                public static final double kArmRevolutionsPerDegree = (kArmTotalRevolutions)
                                / kArmTotalDegrees;

                public static final double kArmPositionAdjustmentFactor = 1 / kArmRevolutionsPerDegree;

                // Estimates, fix this once we get exact measurements
                public static final double kShooterPivotTotalDegrees = 360;
                public static final double kShooterPivotTotalRevolutions = 1;

                // Convert angle of travel to encoder rotations
                public static final double kShooterPivotRevolutionsPerDegree = (kShooterPivotTotalRevolutions)
                                / kShooterPivotTotalDegrees;

                public static final double kShooterPivotPositionAdjustmentFactor = 1
                                / kShooterPivotRevolutionsPerDegree;

                // Temporary Arm PID configuration
                public static final double homeArmPosition = 50 + kArmPositionShift;
                public static final double podiumArmPosition = 50 + kArmPositionShift;

                public static final double homeShooterPivotPosition = 40;
                public static final double podiumShooterPivotPosition = 124;
        }

        public static final class ClimberConstants {
                public static final IdleMode kWinchMotorIdleMode = IdleMode.kBrake;

                // PIDS
                // Winch PID coefficients
                public static final int kWinchPIDSlot = 0;
                public static final double kWinchP = 0.01;
                public static final double kWinchI = 0;
                public static final double kWinchD = 0;
                public static final double kWinchFF = 0.001;
                public static final double kWinchMaxOutput = .5;
                public static final double kWinchMinOutput = -.5;
                public static final double kWinchMaxAccel = .1;
                public static final double kWinchMaxVelo = .5;

                public static final double kRaiseSpeed = 1;
                public static final double kLowerSpeed = .5;

                // Encoder SetPoints for different positions - Need to fill with correct values
                public static final double climbMoveToTrapScore = 10;
                public static final double climbAtTrapScore = 20;
                public static final double climbAtTrapFinal = 15;
        }

        public static final class BlinkinConstants {
                public static int pwmPort = 0;

                public enum ledColor {

                        WITHNOTE("With Note", 0.65),
                        TARGETFOUND("Target Found", .69),
                        READYTOSHOOT("Ready to Shoot", .77);

                        public final String label;
                        public final double color;

                        private ledColor(String label, double color) {
                                this.label = label;
                                this.color = color;
                        }

                }

        }

        public static final class DriveConstants {
                // Driving Parameters - Note that these are not the maximum capable speeds of
                // the robot, rather the allowed maximum speeds
                public static final double kMaxSpeedAdjustment = 1;
                public static final double kMaxSpeedMetersPerSecond = 5.75 * kMaxSpeedAdjustment;

                // if you want to slow down the rotation speed, change the adjustment factor
                public static final double kAngularSpeedAdjustment = .95;
                public static final double kMaxAngularSpeed = 2 * Math.PI * kAngularSpeedAdjustment; // radians per
                // second

                public static final double kDirectionSlewRate = 1.2; // radians per second
                public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
                public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

                // Chassis configuration
                public static final double kTrackWidth = Units.inchesToMeters(24.0);
                // Distance between centers of right and left wheels on robot
                public static final double kWheelBase = Units.inchesToMeters(22.0);
                // Distance between front and back wheels on robot
                public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

                // Angular offsets of the modules relative to the chassis in radians
                public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
                public static final double kFrontRightChassisAngularOffset = 0;
                public static final double kBackLeftChassisAngularOffset = Math.PI;
                public static final double kBackRightChassisAngularOffset = Math.PI / 2;

                // SPARK MAX CAN IDs
                public static final int kFrontLeftDrivingCanId = 1;
                public static final int kRearLeftDrivingCanId = 2;
                public static final int kFrontRightDrivingCanId = 3;
                public static final int kRearRightDrivingCanId = 4;

                public static final int kFrontLeftTurningCanId = 11;
                public static final int kRearLeftTurningCanId = 12;
                public static final int kFrontRightTurningCanId = 13;
                public static final int kRearRightTurningCanId = 14;

                // Gyro Constants
                public static final boolean kGyroReversed = false;
        }

        public static final class ModuleConstants {
                // The MAXSwerve module can be configured with one of three pinion gears: 12T,
                // 13T, or 14T.
                // This changes the drive speed of the module (a pinion gear with more teeth
                // will result in a
                // robot that drives faster).
                public static final int kDrivingMotorPinionTeeth = 14;

                // Invert the turning encoder, since the output shaft rotates in the opposite
                // direction of
                // the steering motor in the MAXSwerve Module.
                public static final boolean kTurningEncoderInverted = true;

                // Calculations required for driving motor conversion factors and feed forward
                public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
                public static final double kWheelDiameterMeters = 0.0762;
                public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
                // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
                // teeth on the bevel pinion
                public static final double kDrivingMotorReduction = (45.0 * 22)
                                / (kDrivingMotorPinionTeeth * 15);
                public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps
                                * kWheelCircumferenceMeters)
                                / kDrivingMotorReduction;

                public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                                / kDrivingMotorReduction; // meters
                public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                                / kDrivingMotorReduction) / 60.0; // meters per second

                public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
                public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per
                                                                                                 // second

                public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
                public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

                public static final double kDrivingP = 0.04;
                public static final double kDrivingI = 0;
                public static final double kDrivingD = 0;
                public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
                public static final double kDrivingMinOutput = -1;
                public static final double kDrivingMaxOutput = 1;

                public static final double kTurningP = 1;
                public static final double kTurningI = 0;
                public static final double kTurningD = 0;
                public static final double kTurningFF = 0;
                public static final double kTurningMinOutput = -1;
                public static final double kTurningMaxOutput = 1;

                public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
                public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

                public static final int kDrivingMotorCurrentLimit = 50; // amps
                public static final int kTurningMotorCurrentLimit = 20; // amps
        }

        public static final class OIConstants {
                public static final int kDriverControllerPort = 0;
                public static final double kDriveDeadband = 0.05;
                public static final int kOperatorControllerPort = 1;
        }

        public static final class AutoConstants {
                public static final double kMaxSpeedMetersPerSecond = 3;
                public static final double kMaxAccelerationMetersPerSecondSquared = 3;
                public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
                public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

                public static final double kPXController = 1;
                public static final double kPYController = 1;
                public static final double kPThetaController = 1;

                // Constraint for the motion profiled robot angle controller
                public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

                public static final double kTurnToleranceDeg = 2;
                public static final double kTurnRateToleranceDegPerS = 8;
                public static final double kAutoGyroTolerance = 2;
        }

        public static final class NeoMotorConstants {
                public static final double kFreeSpeedRpm = 5676;
                public static final double kVortexFreeSpeedRpm = 6784;
        }
}
