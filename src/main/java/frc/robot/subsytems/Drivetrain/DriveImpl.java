package frc.robot.subsytems.Drivetrain;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpiutil.math.Pair;

public class DriveImpl extends RepeatingPooledSubsystem implements Drive {

    private final SpeedController leftSide;
    private final SpeedController rightSide;
    private final DifferentialDrive drivetrain;

    public DriveImpl(SpeedController leftSide, SpeedController rightSide) {
        super(20, TimeUnit.MILLISECONDS);
        this.leftSide = leftSide;
        this.rightSide = rightSide;
        this.drivetrain = new DifferentialDrive(leftSide, rightSide);
    }

    @Override
    public void defineResources() {
        require(leftSide);
        require(rightSide);
    }

    @Override
    public void task() throws Exception {
        // TODO Auto-generated method stub
    }

    @Override
    public void driveTank(double leftSpeed, double rightSpeed) {
        this.drivetrain.tankDrive(leftSpeed, rightSpeed);
    }

    @Override
    public void driveArcade(double speed, double rotation) {
        this.drivetrain.arcadeDrive(speed, rotation);
    }

    @Override
    public double getHeading() {
        return 0;
    }

    @Override
    public void resetEncoderTicks() {
        // TODO Auto-generated method stub
    }

    @Override
    public Pair<Double, Double> distanceDriven() {
        // TODO Auto-generated method stub
        return Pair.of(0.0, 0.0);
    }
}
