package frc.robot.subsystems.BallPath;

import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.subsystem.Subsystem;

public interface BallPath extends Subsystem, LifecycleListener {
    // Declare interface with team
    void startIntake();
    void reverseIntake();
    void stopIntake();
    // checks if ball is at bottom of elevator, checks sensors and primes
    boolean checkIfPrime();
    // can be used to stop a ball from going up the elevator in the event that we cannot shoot the ball
    void reverseElevator();
    void findAndCenterTarget();
    boolean readyToShoot();
    void startShooter();
    void stopShooter();
}
