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
import frc.robot.maps.RobotMap.DriveMap;

public class Drive extends SmartSubsystemBase {
    // Creating my kinematics object using the module locations
    SwerveDriveKinematics kinematics;
    double maxDriveSpeedMetersPerSecond;
    double maxRotationRadiansPerSecond;
    Gyro gyro;

    public Drive(DriveMap map) {
        kinematics = new SwerveDriveKinematics(map.frontLeftLocation(), map.frontRightLocation(),
                map.rearLeftLocation(), map.rearRightLocation());
        gyro = map.gyro();
        maxDriveSpeedMetersPerSecond = map.maxDriveSpeedMetersPerSecond();
        maxRotationRadiansPerSecond = map.maxRotationRadianPerSecond();
    }

    public CommandBase fieldCentricDrive(final DoubleSupplier translateX, final DoubleSupplier translateY,
            final DoubleSupplier rotation) {
        return running("Field Centric Drive", () -> {
            // Need to convert inputs from -1..1 scale to m/s
            double translateXSpeed = translateX.getAsDouble() * maxDriveSpeedMetersPerSecond;
            double translateYSpeed = translateY.getAsDouble() * maxDriveSpeedMetersPerSecond;
            double rotationSpeed = rotation.getAsDouble() * maxRotationRadiansPerSecond;

            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translateXSpeed, translateYSpeed,
                    rotationSpeed, Rotation2d.fromDegrees(gyro.getAngle()));

            // Now use this in our kinematics
            SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

            // Front left module state
            SwerveModuleState frontLeft = moduleStates[0];

            // Front right module state
            SwerveModuleState frontRight = moduleStates[1];

            // Back left module state
            SwerveModuleState rearLeft = moduleStates[2];

            // Back right module state
            SwerveModuleState rearRight = moduleStates[3];
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
