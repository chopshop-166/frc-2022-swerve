// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.outputs.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.maps.RobotMap.SpindexerMap;

public class Spindexer extends SmartSubsystemBase {

    private final static double SPINDEXER_SPEED = 0.5;
    private final static double WASHER_MACHINE_DISTANCE = 0.25;
    private final static double WASHER_MACHINE_SPEED = 0.2;

    private final SmartMotorController motor;
    private final IEncoder encoder;

    public Spindexer(final SpindexerMap map) {
        super();
        motor = map.getMotor();
        encoder = motor.getEncoder();
    }

    // Rotates the Spindexer while command is running
    // Should be used in "raceWith" or similar to control
    public CommandBase spin() {
        return startEnd("SpinToShoot", () -> {
            motor.set(SPINDEXER_SPEED);
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
            motor.set(Math.signum(distance) * WASHER_MACHINE_SPEED);
        }, () -> Math.abs(encoder.getDistance()) >= Math.abs(distance));
    }

    // This command helps the Power Cells settle into the spindexer by jostling them
    // around.
    public CommandBase washerMachine() {
        return new SequentialCommandGroup(spinDistance(WASHER_MACHINE_DISTANCE),
                spinDistance(-WASHER_MACHINE_DISTANCE));
    }

    @Override
    public void reset() {
        encoder.reset();
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
