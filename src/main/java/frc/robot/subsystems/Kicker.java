package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.outputs.SmartMotorController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.maps.RobotMap.KickerMap;

public class Kicker extends SmartSubsystemBase {

    private static final double KICKER_SPEED = 1;
    private static final double KICKER_SPEED_REVERSE = -0.5;
    private final SmartMotorController motor;
    private final BooleanSupplier ballSensor;

    // Variables to track how many balls we've shot
    // Commands may reset this count during operation
    private int ballCount;
    private boolean prevBallValue;

    public Kicker(final KickerMap map) {
        super();
        motor = map.getMotor();
        ballSensor = map.getBallSensor();
    }

    // Command to bring a specified number of balls into the shooter
    // This can "raceWith" the spindexer to control shooting balls in auto
    public CommandBase KickToShoot(final int numBalls) {
        return functional("KickToShoot", () -> {
            resetBallCount();
        }, () -> {
            motor.set(KICKER_SPEED);
        }, (interrupted) -> {
            motor.set(0);
        }, () -> {
            return ballCount >= numBalls;
        });
    }

    // Fallback command if the sensor isn't working
    public CommandBase run(final boolean up) {
        return startEnd("RunKicker", () -> {
            motor.set(up ? KICKER_SPEED : KICKER_SPEED_REVERSE);
        }, () -> {
            motor.set(0);
        });
    }

    public CommandBase start() {
        return new SequentialCommandGroup(new InstantCommand(() -> {
            motor.set(KICKER_SPEED);
        }, this), new WaitCommand(1)).withName("Start Kicker");
    }

    public CommandBase stop() {
        return instant("stop Kicker", () -> {
            motor.set(0);
        });
    }

    @Override
    public void reset() {
        resetBallCount();
    }

    private void resetBallCount() {
        ballCount = 0;
    }

    // Track how many balls we've shot
    // Increments when the ball leaves the sensor range
    // This should happen when the ball has entered the shooter and can be
    // considered "shot"
    private void periodicBallCount() {
        final boolean currentBallValue = ballSensor.getAsBoolean();
        if (!currentBallValue && prevBallValue) {
            ballCount++;
        }
        prevBallValue = currentBallValue;
    }

    @Override
    public void periodic() {
        periodicBallCount();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
