// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.outputs.SwerveModule;
import frc.robot.maps.RobotMap.DriveMap;

public class Drive extends SmartSubsystemBase {
    // Creating my kinematics object using the module locations
    private final SwerveDriveKinematics kinematics;
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule rearLeft;
    private final SwerveModule rearRight;

    private final double maxDriveSpeedMetersPerSecond;
    private final double maxRotationRadiansPerSecond;
    private final Gyro gyro;

    public Drive(final DriveMap map) {
        super();

        frontLeft = map.frontLeft();
        frontRight = map.frontRight();
        rearLeft = map.rearLeft();
        rearRight = map.rearRight();
        kinematics = new SwerveDriveKinematics(frontLeft.getLocation(), frontRight.getLocation(),
                rearLeft.getLocation(), rearRight.getLocation());
        gyro = map.gyro();
        maxDriveSpeedMetersPerSecond = map.maxDriveSpeedMetersPerSecond();
        maxRotationRadiansPerSecond = map.maxRotationRadianPerSecond();
    }

    public CommandBase fieldCentricDrive(final DoubleSupplier translateX, final DoubleSupplier translateY,
            final DoubleSupplier rotation) {
        return running("Field Centric Drive", () -> {
            // Need to convert inputs from -1..1 scale to m/s
            final double translateXSpeed = translateX.getAsDouble() * maxDriveSpeedMetersPerSecond;
            final double translateYSpeed = translateY.getAsDouble() * maxDriveSpeedMetersPerSecond;
            final double rotationSpeed = rotation.getAsDouble() * maxRotationRadiansPerSecond;

            final ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translateXSpeed, translateYSpeed,
                    rotationSpeed, Rotation2d.fromDegrees(gyro.getAngle()));

            // Now use this in our kinematics
            final SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

            // Front left module state
            frontLeft.setDesiredState(moduleStates[0]);

            // Front right module state
            frontRight.setDesiredState(moduleStates[1]);

            // Back left module state
            rearLeft.setDesiredState(moduleStates[2]);

            // Back right module state
            rearRight.setDesiredState(moduleStates[3]);
        });
    }

    // Reset the gyro heading in case it has drifted significantly.
    public CommandBase resetGyro() {
        return instant("ResetGyro", () -> {
            gyro.reset();
        });
    }

    @Override
    public void reset() {
        // Nothing to reset here
        gyro.reset();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Use this for any background processing
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
