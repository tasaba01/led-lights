package frc.robot.subsystems.BallPath.Intake;

import ca.team3161.lib.robot.subsystem.Subsystem;

public interface Intake extends Subsystem{
    // outer roller will run on extend and stop on retract
    // void extendOuter();
    // void retractOuter();
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
}
