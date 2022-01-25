package frc.robot.subsystems.BallPath;

import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.subsystem.Subsystem;

public interface BallPath extends Subsystem, LifecycleListener {
    // Declare interface with team
    void extendOuter();
    void retractOuter();
    // checks colour of ball
    boolean checkColour();
    // checks if ball is held by intake
    boolean checkBall();
    void reverse();
    // checks if ball is at bottom of elevator
    boolean checkIfPrime();
    // primes a ball
    void runInnerleft();
    void runInnerRight();
    void stopInner();
    // Assuming current design of 2 to 3 rollers in the robot.
    // Checks if balls are primed and where they are primed; ex. one at bottom of elevator and one in left intake
    int[] checkBalls();
    void startElevator();
    void stopElevator();
    // can be used to stop a ball from going up the elevator in the event that we cannot shoot the ball
    void reverseElevator();
    double findTarget();
    void centerTarget(double tx);
    void getDistance(double ty, double angle1, double angle2);
    // runs flywheel
    boolean readyToShoot();
    void setHoodAngle(double distance);
    void startShooter();
    void stopShooter();
}
