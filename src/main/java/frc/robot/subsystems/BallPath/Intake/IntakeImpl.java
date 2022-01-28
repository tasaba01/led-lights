package frc.robot.subsystems.BallPath.Intake;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;

import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class IntakeImpl extends RepeatingPooledSubsystem implements Intake {

    private final WPI_TalonSRX intake;
    private boolean primed;
    private int speed = 1;
    private ColorSensorV3 leftColorSensor;
    private ColorSensorV3 rightColorSensor;
    private final ColorMatch colorMatcher = new ColorMatch();
    private Color leftDetected;
    private Color rightDetected;
    private final Color target = new Color(1, 1, 1);


    public IntakeImpl(WPI_TalonSRX intake, ColorSensorV3 leftColorSensor, ColorSensorV3 rightColorSensor) {
        super(20, TimeUnit.MILLISECONDS);
        this.intake = intake;
        this.colorMatcher.addColorMatch(this.target);
    }

    @Override
    public void defineResources() {
        require(intake);
    }

    @Override
    public void task() throws Exception {
        // TODO Auto-generated method stub
    }

    @Override
    public void start(){
        this.intake.set(this.speed);
        this.checkIntake();
    }

    @Override
    public void reverse(){
        this.intake.set(-this.speed);
    }

    @Override
    public void stop(){
        this.intake.set(0);
    }


    // checks if ball is at bottom of elevator
    @Override
    public boolean checkIfPrimed(){
        // if (ballUnderElevator){
        //     this.primed = true;
        // }

        return this.primed;
    }
    
    @Override
    public boolean checkColour(){
        this.leftDetected = this.leftColorSensor.getColor();
        this.rightDetected = this.rightColorSensor.getColor();
        ColorMatchResult leftResult = this.colorMatcher.matchColor(leftDetected);
        ColorMatchResult rightResult = this.colorMatcher.matchColor(rightDetected);

        if (leftResult.color == this.target || rightResult.color == this.target){
            return true;
        } 

        return false;
    }

    @Override
    public boolean checkIntake(){
        // Calls check color
        return true;
    }

    // primes a ball
    // @Override
    // public void runInner(int side){
    //     // side: 0 = front
    //     // side: 1 = back
    //     if (side == 0){
    //         this.frontIntake.set(1);
    //     } else if (side == 1){
    //         this.backIntake.set(1);
    //     }
    // }

    // @Override
    // public void stopInner(int side){
    //     // side: 0 = front
    //     // side: 1 = back
    //     if (side == 0){
    //         this.frontIntake.set(0);
    //     } else if (side == 1){
    //         this.backIntake.set(0);
    //     }
    // }
}
