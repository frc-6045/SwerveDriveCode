package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule 
{
    
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    // absolute encoder is so the robot knows its position even when the power on the robot turns off since this will store that data while the built in encoders in the motors will lose their heading. this will provide a constant refernce so all the wheels can be synced up. 
    // absolute encoder is plugged into the analog input area on the RIO
    // since the starting value of each encoder may be off when you put the robot together, one encoder may start at 45 degrees and another at 90 even if they are facing the same direction, you can apply a known offset to each wheel so that everything is alligned properly

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    // a constructor to assign the values to each of the modules
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed)
    {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(Constants.moduleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(Constants.moduleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(Constants.moduleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(Constants.moduleConstants.kTurningEncoderRPM2RadPerSec);
        
        //the proportional constant usually does a good enough job alread so we dont need the integral or derivative term in this instance
        turningPidController = new PIDController(Constants.moduleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() 
    {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() 
    {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() 
    {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity()
    {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() 
    {
        //the first line will give us how many percent of a full rotation the encoder is reading which we use to allign all the wheels later
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        //The : in this case is a ternary operator that is a much simpler way of writing a if else. in this case, if the encoder is reversed it will return -1, else return 1.
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    // this method will reset the drive encoder to 0 as well as set the turning encoders to equal to their proper heading with respect to the absolute encoder.
    public void resetEncoders() 
    {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    // using this we can more easily access this data from other classes that may require the wheel speed as well as the angle of the wheel
    public SwerveModuleState getState() 
    {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001)
        {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop()
    {
        driveMotor.set(0);
        turningMotor.set(0);
    }


}
