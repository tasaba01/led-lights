package frc.robot.subsystems.BallPath.Intake;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;

import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.Ultrasonic;

public class IntakeImpl extends RepeatingPooledSubsystem implements Intake {

    private final WPI_TalonSRX intake;
    private int speed = 1;


    private ColorSensorV3 leftColorSensor;
    private ColorSensorV3 rightColorSensor;
    private final ColorMatch colorMatcher = new ColorMatch();
    private final Color target = new Color(1, 1, 1);
    
    private final Ultrasonic intakeSensor;


    public IntakeImpl(WPI_TalonSRX intake, ColorSensorV3 leftColorSensor, ColorSensorV3 rightColorSensor, Ultrasonic intakeSensor) {
        super(20, TimeUnit.MILLISECONDS);
        this.intake = intake;
        this.leftColorSensor = leftColorSensor;
        this.rightColorSensor = rightColorSensor;
        this.colorMatcher.addColorMatch(this.target);
        this.intakeSensor = intakeSensor;
    }

    @Override
    public void defineResources() {
        require(intake);
        require(intakeSensor);
        require(leftColorSensor);
        require(rightColorSensor);
    }

    @Override
    public void task() throws Exception {
    }

    @Override
    public void start(){
        this.intake.set(this.speed);
    }

    @Override
    public void reverse(){
        this.intake.set(-this.speed);
    }

    @Override
    public void stop(){
        this.intake.set(0);
    }
    
    @Override
    public boolean checkColour(){
        Color leftDetected = this.leftColorSensor.getColor();
        Color rightDetected = this.rightColorSensor.getColor();
        ColorMatchResult leftResult = this.colorMatcher.matchColor(leftDetected);
        ColorMatchResult rightResult = this.colorMatcher.matchColor(rightDetected);

        // if we are red team
        if (leftResult.color == this.target || rightResult.color == this.target){
            // if team ball
            return true;
        } 

        return false;

        // if we are blue team
        // if (leftResult.color == this.target || rightResult.color == this.target){
        //     // if team ball
        //     return false;
        // } 

        // return true;
    }

    @Override
    public boolean checkIntake(){
        if (this.intakeSensor.getRangeInches() <= 10){
            return true;
        }

        return false;
    }
}
