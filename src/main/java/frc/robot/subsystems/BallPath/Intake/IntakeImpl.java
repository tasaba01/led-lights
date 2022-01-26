package frc.robot.subsystems.BallPath.Intake;

public class IntakeImpl {
    public void extendOuter(){};
    public void retractOuter(){};
    // checks colour of ball
    public boolean checkColour(){
        return true;
    };
    // checks if ball is held by intake
    public boolean checkBall(){
        return true;
    };
    public void reverse(){};
    // checks if ball is at bottom of elevator
    public boolean checkIfPrime(){
        return true;
    };
    // primes a ball
    public void runInnerleft(){};
    public void runInnerRight(){};
    public void stopInner(){};
}
