package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveCommands {

        protected static Command rotateRobot(CommandSwerveDrivetrain drive, Supplier<Rotation2d> rotation,
                        Supplier<Double> driveForward,
                        Supplier<Double> driveLeft) {
                final PIDController headingController = new PIDController(7, 0, 0);
                return Commands.run(() -> {

                        headingController.enableContinuousInput(-Math.PI, Math.PI);

                        final var speeds = new ChassisSpeeds(
                                        driveForward.get() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
                                                        * Constants.kLinearSpeedMultiplier,
                                        driveLeft.get() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
                                                        * Constants.kLinearSpeedMultiplier,
                                        0);

                        double fieldRelativeAngle = drive.getPose().getRotation().getRadians();

                        speeds.omegaRadiansPerSecond = headingController.calculate(fieldRelativeAngle,
                                        rotation.get().getMeasure().in(Radians));

                        drive.setControl(new SwerveRequest.FieldCentric()
                                        .withVelocityX(speeds.vxMetersPerSecond)
                                        .withVelocityY(speeds.vyMetersPerSecond)
                                        .withRotationalRate(speeds.omegaRadiansPerSecond));
                }, drive);
        }

        protected static Command rotateRobot(CommandSwerveDrivetrain drive, Supplier<Rotation2d> rotation) {
                return rotateRobot(drive, rotation, () -> Double.valueOf(0), () -> Double.valueOf(0));
        }

        private static Command joyStickDrive(CommandSwerveDrivetrain drive, Supplier<Double> driveForward,
                        Supplier<Double> driveLeft, Supplier<Double> driveRotation) {
                return Commands.run(() -> {
                        ChassisSpeeds speeds = new ChassisSpeeds(
                                        driveForward.get()
                                                        * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
                                                        * Constants.kLinearSpeedMultiplier,

                                        driveLeft.get() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
                                                        * Constants.kLinearSpeedMultiplier,

                                        Math.copySign(
                                                        driveRotation.get() * driveRotation.get()
                                                                        * Constants.kAngularMaxSpeed
                                                                                        .in(RadiansPerSecond)
                                                                        * Constants.kAngluarSpeedMultiplier,
                                                        driveRotation.get()));

                        drive.setControl(new SwerveRequest.FieldCentric()
                                        .withVelocityX(speeds.vxMetersPerSecond)
                                        .withVelocityY(speeds.vyMetersPerSecond)
                                        .withRotationalRate(speeds.omegaRadiansPerSecond));
                }, drive);
        }

        private static Command snakeDrive(CommandSwerveDrivetrain drive, Supplier<Double> driveForward,
                        Supplier<Double> driveLeft, Supplier<Double> driveRotation) {
                return Commands.run(() -> {
                        final PIDController headingController = new PIDController(100, 0, 0);
                        final boolean snakeDriveActive = !(Math.abs(driveRotation.get()) > 0);

                        ChassisSpeeds speeds;
                        if (snakeDriveActive) {
                                headingController.enableContinuousInput(-Math.PI, Math.PI);

                                speeds = new ChassisSpeeds(
                                                driveForward.get()
                                                                * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
                                                                * Constants.kLinearSpeedMultiplier,
                                                driveLeft.get()
                                                                * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
                                                                * Constants.kLinearSpeedMultiplier,
                                                0);

                                Angle headingAngle = Radians
                                                .of(Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond)
                                                                + Math.PI);

                                if (Math.abs(speeds.vyMetersPerSecond) > 0.1
                                                || Math.abs(speeds.vxMetersPerSecond) > 0.1) {
                                        speeds.omegaRadiansPerSecond = headingController.calculate(
                                                        drive.getPose().getRotation().getRadians(),
                                                        headingAngle.in(Radians));
                                }
                        } else {
                                speeds = new ChassisSpeeds(
                                                driveForward.get()
                                                                * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
                                                                * Constants.kLinearSpeedMultiplier,
                                                driveLeft.get()
                                                                * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
                                                                * Constants.kLinearSpeedMultiplier,
                                                Math.copySign(
                                                                driveRotation.get()
                                                                                * driveRotation.get()
                                                                                * Constants.kAngularMaxSpeed
                                                                                                .in(RadiansPerSecond)
                                                                                * Constants.kAngluarSpeedMultiplier,
                                                                driveRotation.get()));
                        }

                        drive.setControl(new SwerveRequest.FieldCentric()
                                        .withVelocityX(speeds.vxMetersPerSecond)
                                        .withVelocityY(speeds.vyMetersPerSecond)
                                        .withRotationalRate(speeds.omegaRadiansPerSecond));

                        headingController.close();
                }, drive);
        }

        protected static Command driveCommand(CommandSwerveDrivetrain drive, Supplier<Double> driveForward,
                        Supplier<Double> driveLeft, Supplier<Double> driveRotation) {
                return Constants.kUseWeirdSnakeDrive ? snakeDrive(drive, driveForward, driveLeft, driveRotation)
                                : joyStickDrive(drive, driveForward, driveLeft, driveRotation);
        }

}
