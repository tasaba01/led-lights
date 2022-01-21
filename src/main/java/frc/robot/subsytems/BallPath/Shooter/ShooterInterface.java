package frc.robot.subsytems.BallPath.Shooter;

import ca.team3161.lib.robot.subsystem.Subsystem;

public interface ShooterInterface extends Subsystem{
    double findTarget();
    void centerTarget(double tx);
    void getDistance(double ty, double angle1, double angle2);
    // runs flywheel
    boolean readyToShoot();
    int checkBalls();
    void setHoodAngle(double distance);
    void startShooter();
    void stopShooter();
}
