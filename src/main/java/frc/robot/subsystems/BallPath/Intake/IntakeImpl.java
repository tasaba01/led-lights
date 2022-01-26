package frc.robot.subsystems.BallPath.Intake;

import java.util.concurrent.TimeUnit;

// import ca.team3161.lib.robot.subsystem.RepeatingIndependentSubsystem;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;

public class IntakeImpl extends RepeatingPooledSubsystem implements Intake {
    public IntakeImpl() {
        super(20, TimeUnit.MILLISECONDS);
        // TODO Auto-generated constructor stub
    }

    @Override
    public void extendOuter(){}

    @Override
    public void retractOuter(){}
    // checks colour of ball

    @Override
    public boolean checkColour(){
        return true;
    }

    // checks if ball is held by intake
    @Override
    public boolean checkBall(){
        return true;
    }

    @Override
    public void reverse(){}

    // checks if ball is at bottom of elevator
    @Override
    public boolean checkIfPrime(){
        return true;
    }

    // primes a ball
    @Override
    public void runInnerleft(){}

    @Override
    public void runInnerRight(){}

    @Override
    public void stopInner(){}

    public void task(){}

    @Override
    public void defineResources(){}
    
}
