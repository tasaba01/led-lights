package frc.robot.subsystems.Drivetrain;

import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.subsystem.Subsystem;
import edu.wpi.first.math.Pair;

public interface Drive extends Subsystem, LifecycleListener{
    void driveTank(double leftSpeed, double rightSpeed);
    void driveArcade(double speed, double rotation);
    void drivePidTank(double leftSpeed, double rotation);
    double getHeading();
    void resetEncoderTicks();
    Pair<Double, Double> distanceDriven();

    Pair<Double, Double> getEncoderTicks();
}

