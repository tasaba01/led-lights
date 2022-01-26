package frc.robot.subsystems.BallPath.Intake;

import ca.team3161.lib.robot.subsystem.Subsystem;
// import com.revrobotics.ColorSensorV3;


public interface Intake extends Subsystem{
    // checks colour of ball
    void start();
    void reverse();
    void stop();

    boolean checkColour();
    // checks if ball is at bottom of elevator
    boolean checkIfPrimed();
    // Assuming current design of 2 to 3 rollers in the robot.
}
