// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.outputs.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.ShooterMap;

public class Shooter extends SmartSubsystemBase {

    private final static double MAX_SPEED = 8500;

    private final static double ROLLER_SPEED = 3000;

    private final SmartMotorController motor;
    private final IEncoder shooterEncoder;
    private final SmartMotorController roller;
    private final IEncoder rollerEncoder;
    private final SmartMotorController hood;

    public Shooter(final ShooterMap map) {
        super();
        motor = map.getMotor();
        shooterEncoder = motor.getEncoder();
        roller = map.getRoller();
        rollerEncoder = roller.getEncoder();
        hood = map.getHood();
    }

    public enum Speed {
        SPIT_OUT(2000, "Spit Out"), GOAL_BASE(3300, "Goal Base"), INITIATION_LINE(3500, "Initiation Line"),
        TRENCH_SHOT(4500, "Trench Shot"), MAX_SPEED(8500, "Max Speed");

        private double speedRPM;
        private String name;

        public double value() {
            return this.speedRPM;
        }

        public String getName() {
            return this.name;
        }

        Speed(final double speedRPM, final String name) {
            this.speedRPM = speedRPM;
            this.name = name;
        }
    }

    public CommandBase spinUpToSpeed(final String name, final DoubleSupplier speed) {
        final PersistenceCheck check = new PersistenceCheck(5,
                () -> Math.abs(shooterEncoder.getRate() - Math.min(speed.getAsDouble(), MAX_SPEED)) <= 100.0);
        // The roller check is less restrictive as it should have less impact
        // final PersistenceCheck rollerCheck = new PersistenceCheck(5,
        // () -> Math.abs(rollerEncoder.getRate() - ROLLER_SPEED) <= 200.0);
        return initAndWait(name, () -> {
            motor.setSetpoint(Math.min(speed.getAsDouble(), MAX_SPEED));
            SmartDashboard.putNumber("Setpoint", Math.min(speed.getAsDouble(), MAX_SPEED));
            check.reset();
            // rollerCheck.reset();
        }, () -> check.getAsBoolean());// && rollerCheck.getAsBoolean());
    }

    public CommandBase spinUpToSpeed(final Speed speed) {
        return spinUpToSpeed(speed.getName(), speed::value);
    }

    // Stop Motor instead of setting to 0 so that it spins down on its own
    public CommandBase spinDown() {
        return instant("Spin Down", () -> {
            motor.stopMotor();
            roller.stopMotor();
        });
    }

    // Get estimated distance to the target and calculate the shooter speed
    public CommandBase autoSpinUp() {
        // TODO collect data to develop model of speed vs distance
        return spinUpToSpeed("Trench Shot", () -> ROLLER_SPEED);
    }

    @Override
    public void reset() {
        motor.stopMotor();
        roller.stopMotor();
        hood.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Speed", shooterEncoder.getRate());
    }
}
