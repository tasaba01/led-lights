package frc.robot.subsystems.BallPath;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;

// import ca.team3161.lib.robot.subsystem.RepeatingIndependentSubsystem;
// import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;

// import java.util.concurrent.TimeUnit;

// import ca.team3161.lib.robot.LifecycleEvent;
// import frc.robot.subsystems.BallPath.Intake.Intake;
// import frc.robot.subsystems.BallPath.Intake.IntakeImpl;
// import frc.robot.subsystems.BallPath.Elevator.ElevatorImpl;
// import frc.robot.subsystems.BallPath.Elevator.Elevator;
// import frc.robot.subsystems.BallPath.Shooter.Shooter;
// import frc.robot.subsystems.BallPath.Shooter.ShooterImpl;

public class BallPathImpl extends RepeatingPooledSubsystem implements BallPath {
    public BallPathImpl() {
        super(20, TimeUnit.MILLISECONDS);
        // TODO Auto-generated constructor stub
    }

    // Declare interface with team
    @Override
    public void extendOuter(){}
    
    @Override
    public void retractOuter(){}

    @Override
    public void reverse(){}
    
    // checks if ball is at bottom of elevator, checks sensors and primes
    @Override
    public boolean checkIfPrime(){
        return true;
    }
    
    // can be used to stop a ball from going up the elevator in the event that we cannot shoot the ball
    @Override
    public void reverseElevator(){}

    @Override
    public void findAndCenterTarget(){}

    @Override
    public boolean readyToShoot(){
        return true;
    }

    @Override
    public void startShooter(){}

    @Override
    public void stopShooter(){}

    @Override
    public void defineResources(){}

    public void task(){}
        
    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {}
   
}
