package frc.robot.subsystems.BallPath.Elevator;

import ca.team3161.lib.robot.subsystem.Subsystem;

public interface Elevator extends Subsystem{
    void setAction(ElevatorAction action);
    boolean ballPrimed();

    enum ElevatorAction {
        NONE,
        PRIME,
        REJECT,
        FEED,
        ;
    }
}
