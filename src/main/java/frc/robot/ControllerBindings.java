package frc.robot;

import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechButton;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechControl;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechAxis;
// import ca.team3161.lib.utils.controls.LogitechDualAction.DpadDirection;

public final class ControllerBindings {
    public static final LogitechAxis Y_AXIS = LogitechAxis.Y;
    public static final LogitechAxis X_AXIS = LogitechAxis.X;

    public static final LogitechControl RIGHT_STICK = LogitechControl.RIGHT_STICK;
    public static final LogitechControl LEFT_STICK = LogitechControl.LEFT_STICK;

    public static final LogitechButton INTAKE_START = LogitechButton.RIGHT_BUMPER;
    public static final LogitechButton INTAKE_STOP = LogitechButton.LEFT_BUMPER;
    public static final LogitechButton INTAKE_REVERSE = LogitechButton.B;

    public static final LogitechButton SPIN_UP = LogitechButton.LEFT_TRIGGER;
    public static final LogitechButton SHOOT = LogitechButton.RIGHT_TRIGGER;

    public static final LogitechButton CLIMBER_EXTEND = LogitechButton.Y;
    public static final LogitechButton CLIMBER_RETRACT = LogitechButton.A;
    public static final LogitechButton CLIMBER_ROTATE = LogitechButton.X;


    
}
