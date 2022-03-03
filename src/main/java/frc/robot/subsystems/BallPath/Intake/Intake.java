package frc.robot.subsystems.BallPath.Intake;

import ca.team3161.lib.robot.subsystem.Subsystem;

public interface Intake extends Subsystem{
    void setAction(IntakeAction action);
    boolean ballPrimed();

    enum IntakeAction {
        NONE,
        PRIME,
        REJECT,
        FEED,
        ;
    }
}
