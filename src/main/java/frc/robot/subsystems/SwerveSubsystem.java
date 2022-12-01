package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase
{
    private final SwerveModule frontLeft = new SwerveModule(
        Constants.DriveConstants.kFrontLeftDriveMotorPort,
        Constants.DriveConstants.kFrontLeftTurningMotorPort,
        Constants.DriveConstants.kFrontLeftDriveEncoderReversed,
        Constants.DriveConstants.kFrontLeftTurningEncoderReversed,
        Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        Constants.DriveConstants.kFrontRightDriveMotorPort,
        Constants.DriveConstants.kFrontRightTurningMotorPort,
        Constants.DriveConstants.kFrontRightDriveEncoderReversed,
        Constants.DriveConstants.kFrontRightTurningEncoderReversed,
        Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        Constants.DriveConstants.kBackLeftDriveMotorPort,
        Constants.DriveConstants.kBackLeftTurningMotorPort,
        Constants.DriveConstants.kBackLeftDriveEncoderReversed,
        Constants.DriveConstants.kBackLeftTurningEncoderReversed,
        Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
        Constants.DriveConstants.kBackRightDriveMotorPort,
        Constants.DriveConstants.kBackRightTurningMotorPort,
        Constants.DriveConstants.kBackRightDriveEncoderReversed,
        Constants.DriveConstants.kBackRightTurningEncoderReversed,
        Constants.DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        Constants.DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        Constants.DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
    
    //this requires another library but I dont know if we are using this gyro or not
    // private AHRS gyro = new AHRS(Spi.port.kMXP);
    // so we will use the old one that i know we have
    private ADIS16470_IMU gyro = new ADIS16470_IMU();

    public SwerveSubsystem()
    {
        //This is a mix of Lambdas and multiThreading. What is going on here is that we are creating a thread for this code to run 
        //on in the background. a random core on the cpu will run through the constructor while the rest of the code compiles. the 
        //gyro needs to take a moment to calibrate itself when we boot up, and so it can't be called imediatley when the swerve 
        //subsystem is called, so we use multithreading to pause the execution of the code in our constructor to wait for it to 
        //recalibrate but the way it is set up, other code can compile while we wait for the heading to be reset to zero.
        new Thread(() ->
        {
            try 
            {
                Thread.sleep(1000);
            } catch (Exception e)
            {
                zeroHeading();
            };
        }).start();
    }

    public void zeroHeading() 
    {
        gyro.reset();
    }

    public double getHeading()
    {
        //this will clamp the heading to positive and negative 180 degrees for the range, 
        //go look it up if you want to know how the function works. its kinda simple but I have no clue where they 
        //found this method
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }
    // other methods like to be able to get the rotation in 2 dimensions which is equivalent to placing the value along a sine curve. 
    //Ask if you would like this to be elaborted on
    public Rotation2d getRotation2d() 
    {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }

    public void stopModules()
    {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    //this is here so that we can set the desired speeds and angles for all the modules at once
    public void setModuleStates(SwerveModuleState[] desiredStates)
    {
        //this top line is used to normalize the wheel speeds so that they are all achievalble. so for example, to get a certain 
        //movement the math may work out to needing one of the wheels to drive at 7 meters per second, but that isn't physically 
        //possible so we need to scale all the values down proportionally untill they are all achievable. 
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
