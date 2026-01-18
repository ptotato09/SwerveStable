package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.swerveUtil.SwerveConstants;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class Constants {
	public static final double stickDeadband = 0.08;

	public static final class Swerve {
		public static final double kSlewRateLimit = 1.5; // Unidades por segundo (de 0 a 1)

		// Spark Max Idle Modes
		public static final SparkMaxConfig.IdleMode driveIdleMode = SparkMaxConfig.IdleMode.kBrake;
		public static final SparkMaxConfig.IdleMode angleIdleMode = SparkMaxConfig.IdleMode.kBrake;

		public static final SparkMaxConfig.IdleMode intakeIdleMode = SparkMaxConfig.IdleMode.kBrake;

		// Spark Max Configs
		public static final SparkMaxConfig driveConfig = new SparkMaxConfig();
		public static final SparkMaxConfig angleConfig = new SparkMaxConfig();

		// Max Output Powers
		public static final double drivePower = 1;
		public static final double anglePower = 1;

		// Gyro
		public static final boolean invertGyro = false;

		// Swerve Module Type
		public static final SwerveConstants chosenModule = SwerveConstants.SDSMK4i
		(SwerveConstants.driveGearRatios.SDSMK4i_L2);

		/* Angle Encoder Invert */
		public static final boolean canCoderInvert = chosenModule.canCoderInvert;
		public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

		/* Motor Inverts */
		public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
		public static final double angleGearRatio = chosenModule.angleGearRatio;

		//Ratios
		public static final double DegreesPerTurnRotation = 360 / angleGearRatio;
		public static final double driveGearRatio = chosenModule.driveGearRatio;

		// meters per rotation
		public static final double wheelCircumference = chosenModule.wheelCircumference;
		public static final double driveRevToMeters = wheelCircumference / (driveGearRatio);
		public static final double driveRpmToMetersPerSecond = driveRevToMeters / 60;

		/* Drivetrain Constants */
		public static final double trackWidth = Units.inchesToMeters(22);
		public static final double wheelBase = Units.inchesToMeters(26);
	
		 //Swerve Kinematics
		public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
				new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
				new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
				new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
				new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

		/* Swerve Current Limiting */
		public static final int angleContinuousCurrentLimit = 20;
		public static final int anglePeakCurrentLimit = 40;
		public static final double anglePeakCurrentDuration = 0.1;
		public static final boolean angleEnableCurrentLimit = true;
		public static final int driveContinuousCurrentLimit = 35;
		public static final int drivePeakCurrentLimit = 60;
		public static final double drivePeakCurrentDuration = 0.1;
		public static final boolean driveEnableCurrentLimit = true;

		/* Angle Motor PID Values */
		public static final double angleKP = 0.01;
		public static final double angleKI = 0;
		public static final double angleKD = 0.00125;
		// public static final double angleKFF = 0;

		/* Angle motor FeedForward Values */
		public static final double angleKS = 0.2;
		public static final double angleKV = 0.1;
		public static final double angleKA = 0.0;

		/* Drive Motor info */
		public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;

		public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * wheelCircumference)
				/ driveGearRatio;

		/* Drive Motor PID Values */
		public static final double driveKP = 0.02;
		public static final double driveKI = 0.0;
		public static final double driveKD = 0.005;
		// private static final double kFFactor = 0.5;
		// public static final double driveKFF = (1 / kDriveWheelFreeSpeedRps) * kFFactor;

		/* Drive Motor Feedforward Values (Ejemplos típicos) */
		public static final double driveKS = 0.15; // Voltios para empezar a mover
		public static final double driveKV = 2.12; // Voltios por metro/segundo
		public static final double driveKA = 0.25; // Voltios por metro/segundo^2

		/** Meters per Second */
		public static final double maxSpeed = 3.658;

		/** Radians per Second */
		public static final double maxAngularVelocity = 5.0;
		public static double angleRampRate = 0;

//////////////Swerve Module Constants////////////
		public static class Modules {

			/* Front Left Module - Module 1 */
			public static final class Mod0 {
				public static final int driveMotorID = 11;
				public static final int angleMotorID = 12;
				public static final int canCoderID = 1;
				public static final Rotation2d angleOffset = Rotation2d.fromDegrees(11.52);
				public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(driveMotorID,
						angleMotorID, canCoderID, angleOffset);
			}

			/* Front Right Module - Module 2 */
			public static final class Mod1 {
				public static final int driveMotorID = 21;
				public static final int angleMotorID = 22;
				public static final int canCoderID = 2;
				public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-33.12);
				public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(driveMotorID,
						angleMotorID, canCoderID, angleOffset);
			}

			/* Back Left Module - Module 3 */
			public static final class Mod2 {
				public static final int driveMotorID = 31;
				public static final int angleMotorID = 32;
				public static final int canCoderID = 3;
				public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-90.36);
				public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(driveMotorID,
						angleMotorID, canCoderID, angleOffset);
			}

			/* Back Right Module - Module 4 */
			public static final class Mod3 {
				public static final int driveMotorID = 41;
				public static final int angleMotorID = 42;
				public static final int canCoderID = 4;
				public static final Rotation2d angleOffset = Rotation2d.fromDegrees(158.4);
				public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(driveMotorID,
						angleMotorID, canCoderID, angleOffset);
			}
		}
	}

	public static final class AutoConstants {
		public static final double kMaxSpeedMetersPerSecond = 3;
		public static final double kMaxAccelerationMetersPerSecondSquared = 2;
		public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 16;
		public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 16;

		// Motion profilied robot angle controller
		public static final TrapezoidProfile.Constraints kThetaControllerConstraints 
		= 
		new TrapezoidProfile.Constraints(
				kMaxAngularSpeedRadiansPerSecond,
				kMaxAngularSpeedRadiansPerSecondSquared);
	}

	public static final class NeoMotorConstants {
		public static final double kFreeSpeedRpm = 5676;
	}


}