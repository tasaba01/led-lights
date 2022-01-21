package frc.robot.subsytems.Drivetrain;

import java.util.concurrent.Future;

import ca.team3161.lib.robot.subsystem.Subsystem;

public interface DriveInterface extends Subsystem{
    void driveTank(double leftSpeed, double rightSpeed);
    void driveArcade(double speed, double rotation);
    double getHeading();
    void resetEncoderTicks();
    Future<Double> distanceDriven();
}