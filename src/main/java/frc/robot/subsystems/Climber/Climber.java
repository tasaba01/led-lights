package frc.robot.subsystems.Climber;

// import ca.team3161.lib.robot.subsystem.Subsystem;

public interface Climber{
    void extendOuterClimber();
    void retractOuterClimber();
    // to attach inner, non retractable arm to the bar
    void attachInner();
    void angleOuter(double angle);
}
