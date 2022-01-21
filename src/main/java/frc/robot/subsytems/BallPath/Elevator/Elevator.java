package frc.robot.subsytems.BallPath.Elevator;

import ca.team3161.lib.robot.subsystem.Subsystem;

public interface Elevator extends Subsystem{
    // Checks if balls are primed and where they are primed; ex. one at bottom of elevator and one in left intake
    int[] checkBalls();
    void startElevator();
    void stopElevator();
    // can be used to stop a ball from going up the elevator in the event that we cannot shoot the ball
    void reverseElevator();
}
