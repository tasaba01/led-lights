package frc.robot.subsystems.BallPath.Elevator;

import java.util.ArrayDeque;
import java.util.Queue;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorImpl extends RepeatingPooledSubsystem implements Elevator {

    private static final double MOTOR_SPEED = 1;
    private static final double PRIMED_DIST_THRESHOLD = 2;
    private static final int SAMPLE_COUNT = 1;

    private final WPI_TalonSRX elevator;
    private final Ultrasonic sensor;

    private ElevatorAction action = ElevatorAction.NONE;
    private boolean lastPresent = false;
    private final Queue<Double> sensorSamples;

    public ElevatorImpl(WPI_TalonSRX elevator, Ultrasonic sensor) {
        super(20, TimeUnit.MILLISECONDS);
        this.elevator = elevator;
        this.sensor = sensor;
        this.sensorSamples = new ArrayDeque<>();
    }

    @Override
    public void defineResources() {
        require(elevator);
        require(sensor);
    }

    @Override
    public void setAction(ElevatorAction inputAction) {
        this.action = inputAction;
        // System.out.println("Elevator Action Set to " + this.action);
    }

    @Override
    public boolean ballPrimed() {
        return lastPresent;
    }

    @Override
    public void task() throws Exception {

        double sensorReading = this.sensor.getRangeInches();
        SmartDashboard.putNumber("Elevator Ultrasonic", sensorReading);
        this.sensorSamples.add(sensorReading);
        if (sensorSamples.size() > SAMPLE_COUNT) {
            this.sensorSamples.remove();
        }
        double meanReading = 0;
        for (Double d : sensorSamples) {
            meanReading += d / sensorSamples.size();
        }

        boolean ballPresent = meanReading < PRIMED_DIST_THRESHOLD;
        // boolean stateChanged = ballPresent != lastPresent;

        switch (this.action) {
            case AUTO:
                this.elevator.set(MOTOR_SPEED);
                break;
            case FEED:
                if (ballPresent) {
                    this.elevator.stopMotor();
                } else {
                    this.elevator.set(MOTOR_SPEED);
                }
                break;
            case PRIME:
                if (ballPresent) {
                    this.elevator.set(MOTOR_SPEED);
                } else {
                    this.elevator.stopMotor();
                }
                break;
            case REJECT:
                if (ballPresent) {
                    this.elevator.set(-MOTOR_SPEED);
                } else {
                    this.elevator.stopMotor();
                }
                break;
            case TEST:
                this.elevator.set(MOTOR_SPEED);
                break;
            case NONE:
            default:
                elevator.stopMotor();
                break;
        }

        lastPresent = ballPresent;
    }

    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
        switch (current) {
            case ON_INIT:
            case ON_AUTO:
            case ON_TELEOP:
            case ON_TEST:
                this.start();
                break;
            case ON_DISABLED:
            case NONE:
            default:
                this.cancel();
                break;
        }
    }
}
