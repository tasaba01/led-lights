package frc.robot.subsystems.Drivetrain;

import ca.team3161.lib.robot.subsystem.Subsystem;
import edu.wpi.first.wpiutil.math.Pair;

public interface Drive extends Subsystem{
    void driveTank(double leftSpeed, double rightSpeed);
    void driveArcade(double speed, double rotation);
    double getHeading();
    void resetEncoderTicks();
    Pair<Double, Double> distanceDriven();
}
