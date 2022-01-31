package frc.robot.subsystems.BallPath.Elevator;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ElevatorImpl extends RepeatingPooledSubsystem implements Elevator {
    public final WPI_TalonSRX elevator;

    public ElevatorImpl(WPI_TalonSRX elevator) {
        super(20, TimeUnit.MILLISECONDS);
        this.elevator = elevator;
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
