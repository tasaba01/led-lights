package frc.robot.subsystems.BallPath.Intake;

import ca.team3161.lib.robot.subsystem.Subsystem;


public interface Intake extends Subsystem{
    // checks colour of ball
    void start();
    void reverse();
    void stop();

    boolean checkIntake();
    boolean checkColour();
    // Assuming current design of 2 to 3 rollers in the robot.
}
