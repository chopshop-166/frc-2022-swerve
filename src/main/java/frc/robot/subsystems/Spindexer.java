// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.motors.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.maps.RobotMap.SpindexerMap;
import frc.utils.SpinDirection;

public class Spindexer extends SmartSubsystemBase {

    private final static double WASHER_MACHINE_DISTANCE = 0.25;

    private final SmartMotorController motor;
    private final IEncoder encoder;

    public enum Speeds {
        SHOOTING(0.3), CLEARING(0.5), WASHER_MACHINE_SPEED(0.4);

        private double value;

        Speeds(double val) {
            value = val;
        }

        double getVal() {
            return value;
        }

    }

    public Spindexer(final SpindexerMap map) {
        super();
        motor = map.getMotor();
        encoder = motor.getEncoder();
    }

    public CommandBase stop() {
        return instant("Stop", () -> motor.set(0));
    }

    // Rotates the Spindexer while command is running
    // Should be used in "raceWith" or similar to control
    public CommandBase spin(final SpinDirection dir, Speeds speed) {
        return startEnd("SpinToShoot", () -> {
            motor.set(dir == SpinDirection.CLOCKWISE ? speed.getVal() : -speed.getVal());
        }, () -> {
            motor.set(0);
        });
    }

    // Bang Bang control to go specified distance (in rotations)
    // Negative distance goes backwards
    // Note this resets the encoder distance
    public CommandBase spinDistance(final double distance) {
        return initAndWait("Spin " + distance + " Rotations", () -> {
            encoder.reset();
            // Determine which direction to move based on the distance we should travel
            motor.set(Math.signum(distance) * Speeds.WASHER_MACHINE_SPEED.getVal());
        }, () -> Math.abs(encoder.getDistance()) >= Math.abs(distance));
    }

    // This command helps the Power Cells settle into the spindexer by jostling them
    // around.
    public CommandBase washerMachine() {
        return new SequentialCommandGroup(stop(), spinDistance(WASHER_MACHINE_DISTANCE), stop(),
                spinDistance(-WASHER_MACHINE_DISTANCE));
    }

    @Override
    public void reset() {
        // encoder.reset();
        motor.set(0);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Enc", encoder.getDistance());
    }

    @Override
    public void safeState() {
        motor.stopMotor();
    }
}
