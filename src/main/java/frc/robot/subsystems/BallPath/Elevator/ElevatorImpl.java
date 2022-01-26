package frc.robot.subsystems.BallPath.Elevator;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;

public class ElevatorImpl extends RepeatingPooledSubsystem implements Elevator {
    public ElevatorImpl() {
        super(20, TimeUnit.MILLISECONDS);
    }

    // temporary, will be changed to something better
    @Override
    public int[] checkBalls(){
        int[] arr = {};
        return arr;
    }

    @Override
    public void startElevator(){}

    @Override
    public void stopElevator(){}
    // can be used to stop a ball from going up the elevator in the event that we cannot shoot the ball

    @Override
    public void reverseElevator(){}

    @Override
    public void defineResources() {}

    public void task(){}
}
