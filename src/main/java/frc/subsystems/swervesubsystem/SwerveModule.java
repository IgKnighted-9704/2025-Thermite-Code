package frc.subsystems.swervesubsystem;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public class SwerveModule{

    //Drive & Angle Motors
    private final TalonFX driveMotor;
    private final SparkMax angleMotor;
    
    //Drive & Angle Encoders
    private final SparkAbsoluteEncoder angleEncoder;

    private final boolean angleMotorReversed;
    private final boolean driveMotorReversed;

    private final PIDController anglePIDController;
    private final PIDController drivePIDController;
    private final SimpleMotorFeedforward drivFeedforward;

    public SwerveModule(int driveMotorID, int angleMotorID, boolean driveMotorReversed, boolean angleMotorReversed){
        this.driveMotor = new TalonFX(driveMotorID);
        this.angleMotor = new SparkMax(angleMotorID, MotorType.kBrushless);

        this.angleEncoder = angleMotor.getAbsoluteEncoder();

        this.angleMotorReversed = angleMotorReversed;
        this.driveMotorReversed = driveMotorReversed;

        //Drive PID Initialization
        drivePIDController = new PIDController(Constants.SwerveConstants.ModuleConstants.kPDriving, 
                                              Constants.SwerveConstants.ModuleConstants.kIDriving,
                                              Constants.SwerveConstants.ModuleConstants.kDDriving);
        //Drive Feedforward Initialization
        drivFeedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.ModuleConstants.kSDriving, 
                                                     Constants.SwerveConstants.ModuleConstants.kVDriving, 
                                                     Constants.SwerveConstants.ModuleConstants.kADriving);
        //Angle PID Initialization 
        anglePIDController = new PIDController(Constants.SwerveConstants.ModuleConstants.kPTurning, 
                                               Constants.SwerveConstants.ModuleConstants.kITurning,
                                               Constants.SwerveConstants.ModuleConstants.kDTurning);

        //Rather then using the max and min input range as constraints, it considers them to be the same point and automatically calculates the shortest route to the setpoint.
        anglePIDController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();

    }

    public double getDrivePosition(){
        return driveMotorReversed ? 
        -1 * (driveMotor.getPosition().getValueAsDouble() * Constants.SwerveConstants.ModuleConstants.kDriveEncoderRot2Meters) : 
        driveMotor.getPosition().getValueAsDouble() * Constants.SwerveConstants.ModuleConstants.kDriveEncoderRot2Meters;
    }

    public double getAngularPosition(){
        return angleMotorReversed ? 
        -1 * angleEncoder.getPosition() * Constants.SwerveConstants.ModuleConstants.kTurningEncoderRot2Rad : 
        angleEncoder.getPosition() * Constants.SwerveConstants.ModuleConstants.kTurningEncoderRot2Rad;
    }

    public double getDriveVelocity(){
        return  driveMotorReversed ? 
        -1 * driveMotor.getVelocity().getValueAsDouble() * Constants.SwerveConstants.ModuleConstants.kDriveEncoderRot2MetersPerSec : 
        driveMotor.getVelocity().getValueAsDouble() * Constants.SwerveConstants.ModuleConstants.kDriveEncoderRot2MetersPerSec;
    }

    public double getAngularVelocity(){
        return angleMotorReversed ? 
        -1 * angleEncoder.getVelocity() * Constants.SwerveConstants.ModuleConstants.kTurningEncoderRot2RadPerSec : 
        angleEncoder.getVelocity() * Constants.SwerveConstants.ModuleConstants.kTurningEncoderRot2RadPerSec;
    }

    public void resetEncoders(){
        driveMotor.setPosition(0.0);
   }

   public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAngularPosition()));
   }

   public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAngularPosition()));
   }

   public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        state.optimize(getState().angle);
            double totalSpeed = drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond) + 
                                drivFeedforward.calculate(state.speedMetersPerSecond);
        driveMotor.set(totalSpeed);
        angleMotor.set(anglePIDController.calculate(getAngularPosition(), state.angle.getRadians()));
   }

   public void stop(){
        driveMotor.set(0.0);
        angleMotor.set(0.0);
   }

}