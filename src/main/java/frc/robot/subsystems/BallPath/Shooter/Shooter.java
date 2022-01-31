package frc.robot.subsystems.BallPath.Shooter;

import ca.team3161.lib.robot.subsystem.Subsystem;

public interface Shooter extends Subsystem{
    void findAndCenterTarget();
    void centerTarget(double tx);
    void getDistance(double ty, double angle1, double angle2);
    // runs flywheel
    boolean readyToShoot();
    int checkBalls();
    void setHoodAngle(double distance);
    void start();
    void stop();
}
