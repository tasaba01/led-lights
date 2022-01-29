package frc.robot.subsystems.BallPath.Elevator;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;

import edu.wpi.first.wpilibj.Ultrasonic;

public class ElevatorImpl extends RepeatingPooledSubsystem implements Elevator {
    public final Ultrasonic elevatorSensor;

    public ElevatorImpl(Ultrasonic elevatorSensor) {
        super(20, TimeUnit.MILLISECONDS);
        this.elevatorSensor = elevatorSensor;
    }

    @Override
    public void start(){}

    @Override
    public void reverse(){}

    @Override
    public void stop(){}
    // can be used to stop a ball from going up the elevator in the event that we cannot shoot the ball

    @Override
    public void defineResources() {}

    public void task(){}
}
