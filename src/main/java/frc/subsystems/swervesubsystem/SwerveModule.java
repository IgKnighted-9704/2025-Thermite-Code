package frc.subsystems.swervesubsystem;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class SwerveModule{

    //Drive & Angle Motors
    private final TalonFX driveMotor;
    private final SparkMax angleMotor;
    
    //Drive & Angle Encoders
    private final SparkAbsoluteEncoder angleEncoder;

    private double angleEncoderPosition;
    private double angleEncoderVelocity;
    private double driveEncoderPosition;
    private double driveEncoderVelocity;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private final PIDController anglePIDController;

    public SwerveModule(int driveMotorID, int angleMotorID, boolean driveMotorReversed, boolean angleMotorReversed, double absoluteEncoderOffsetDeg, boolean absoluteEncoderReversed){
        this.driveMotor = new TalonFX(driveMotorID);
        this.angleMotor = new SparkMax(angleMotorID, MotorType.kBrushless);

        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.absoluteEncoderOffsetRad = Math.toRadians(absoluteEncoderOffsetDeg);

        this.angleEncoder = angleMotor.getAbsoluteEncoder();

        //Encoedr Values
        driveEncoderPosition = driveMotorReversed ? 
            -1 * (driveMotor.getPosition().getValueAsDouble() * Constants.SwerveConstants.ModuleConstants.kDriveEncoderRot2Meters) : 
            driveMotor.getPosition().getValueAsDouble() * Constants.SwerveConstants.ModuleConstants.kDriveEncoderRot2Meters;
        driveEncoderVelocity = driveMotorReversed ? 
            -1 * driveMotor.getVelocity().getValueAsDouble() * Constants.SwerveConstants.ModuleConstants.kDriveEncoderRot2MetersPerSec : 
            driveMotor.getVelocity().getValueAsDouble() * Constants.SwerveConstants.ModuleConstants.kDriveEncoderRot2MetersPerSec;
        angleEncoderPosition = angleMotorReversed ? 
            -1 * angleEncoder.getPosition() * Constants.SwerveConstants.ModuleConstants.kTurningEncoderRot2Rad : 
            angleEncoder.getPosition() * Constants.SwerveConstants.ModuleConstants.kTurningEncoderRot2Rad;
        angleEncoderVelocity = angleMotorReversed ? 
            -1 * angleEncoder.getVelocity() * Constants.SwerveConstants.ModuleConstants.kTurningEncoderRot2RadPerSec : 
            angleEncoder.getVelocity() * Constants.SwerveConstants.ModuleConstants.kTurningEncoderRot2RadPerSec;  


        //Angle PID Initialization 
        anglePIDController = new PIDController(Constants.SwerveConstants.ModuleConstants.kPTurning, 
                                               Constants.SwerveConstants.ModuleConstants.kITurning,
                                               Constants.SwerveConstants.ModuleConstants.kDTurning);

        //Rather then using the max and min input range as constraints, it considers them to be the same point and automatically calculates the shortest route to the setpoint.
        anglePIDController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();

    }


    public double getDrivePosition(){
        return driveEncoderPosition;
    }

    public double getAngularPosition(){
        return angleEncoderPosition;
    }

    public double getDriveVelocity(){
        return driveEncoderVelocity;
    }

    public double getAngularVelocity(){
        return angleEncoderVelocity;
    }

    public void resetEncoders(){
        driveMotor.setPosition(0.0);
   }

   public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAngularPosition()));
   }

   public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        state.optimize(getState().angle);
        driveMotor.set(state.speedMetersPerSecond / Constants.SwerveConstants.ModuleConstants.kPhysicalMaxSpeedMetersPerSecond);
        angleMotor.set(anglePIDController.calculate(getAngularPosition(), state.angle.getRadians()));
   }

   public void stop(){
        driveMotor.set(0.0);
        angleMotor.set(0.0);
   }

}