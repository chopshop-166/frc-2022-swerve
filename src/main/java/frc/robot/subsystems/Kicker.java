package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.outputs.SmartMotorController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.KickerMap;

public class Kicker extends SmartSubsystemBase {

    private static double KICKER_SPEED = 0.75;
    private SmartMotorController motor;
    private BooleanSupplier ballSensor;

    // Variables to track how many balls we've shot
    // Commands may reset this count during operation
    private int ballCount;
    private boolean prevBallValue;

    public Kicker(KickerMap map) {
        motor = map.getMotor();
        ballSensor = map.getBallSensor();
    }

    // Command to bring a specified number of balls into the shooter
    // This can "raceWith" the spindexer to control shooting balls in auto
    public CommandBase KickToShoot(int numBalls) {
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
    public CommandBase runKicker() {
        return startEnd("RunKicker", () -> {
            motor.set(KICKER_SPEED);
        }, () -> {
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
        boolean currentBallValue = ballSensor.getAsBoolean();
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
