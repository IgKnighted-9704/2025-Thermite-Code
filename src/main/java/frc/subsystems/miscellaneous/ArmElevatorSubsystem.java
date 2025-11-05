package frc.subsystems.miscellaneous;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmElevatorConstants.LevelConstants;

public class ArmElevatorSubsystem extends SubsystemBase {

    //Elevator Position Preset
    private enum Preset {
        STOW,
        FUNNEL,
        LOADING,
        L2,
        L3,
        L4,
        L2Score,
        L3Score,
        L4Score,
    }

    //Elevator 
        private final TalonFX elevatorMotorA;
        private final TalonFX elevatorMotorB;
        private final ProfiledPIDController elevatorPIDController;
        private ElevatorFeedforward elevatorFeedforward;


    //Arm
        private final TalonFX armMotor;
        private final SparkMax armEncoderSetup;
        private final SparkAbsoluteEncoder armEncoder;
        private final PIDController armPIDController;
        private SimpleMotorFeedforward armFeedforward;

    //End Effector
        private final TalonFX endEffectorMotor;
    
    //Sensors
    private DigitalInput coralFunnelSensor;
    private DigitalInput coralInEndEffectorSensor;
    
    //Periodic Tracker
        private double desiredArmAngleDeg;
        private double desiredElevInches;
        private Preset currentPreset;
    
    //State Control : Automatic & Manual
        //End Effector
            private boolean autoIntakeActive = false;
            private boolean manualIntakeActive = false;
            private boolean outtake = false;
        //Elevator
            private boolean manualElevator = false;
            private boolean manualArm = false;
        //Sensor 
        private boolean funnelSensor;
        private boolean endEffectorSensor;
    
        //Shuffleboard
            //Live Data
                private final ShuffleboardTab armElevatorLiveDataTab;
                    private final GenericEntry armAngleDeg;
                    private final GenericEntry elevHeightIn;
                    private final GenericEntry armDesiredPos;
                    private final GenericEntry elevDesiredPos;
                    private final GenericEntry elevatorPreset;
                    private final GenericEntry funnelSensorEntry;
                    private final GenericEntry endeffectorSensorEntry;
                    private final GenericEntry state; //debugger purposes only
            // PID & FEEDFORWARD
                private final ShuffleboardTab armElevatorTuneDataTab;
                    //Elevator
                        //PID
                            private final GenericEntry kPElevatorEntry;
                            private final GenericEntry kIElevatorEntry;
                            private final GenericEntry kDElevatorEntry;
                        //Feedforward
                            private final GenericEntry kSElevatorEntry;
                            private final GenericEntry kVElevatorEntry;
                            private final GenericEntry kAElevatorEntry;
                    //Arm
                        //PID                                
                            private final GenericEntry kPArmEntry;
                            private final GenericEntry kIArmEntry;
                            private final GenericEntry kDArmEntry;
                        //Feedforward
                            private final GenericEntry kSArmEntry;
                            private final GenericEntry kVArmEntry;
                            private final GenericEntry kAArmEntry;

    public ArmElevatorSubsystem() {
        //Elevator Motor Initialization
            elevatorMotorA = new TalonFX(Constants.ArmElevatorConstants.StateConstants.kElevatorMotorAID);
            elevatorMotorB = new TalonFX(Constants.ArmElevatorConstants.StateConstants.kElevatorMotorBID);
        //Arm Motor Initialization
            armMotor = new TalonFX(Constants.ArmElevatorConstants.StateConstants.kArmMotorID);
            armEncoderSetup = new SparkMax(Constants.ArmElevatorConstants.StateConstants.kArmAbsoluteEncoderID, MotorType.kBrushless);
            armEncoder = armEncoderSetup.getAbsoluteEncoder();
        //End Effector Motor Initialization
            endEffectorMotor = new TalonFX(Constants.ArmElevatorConstants.StateConstants.kEndEffectorMotorID);
        //Sensors Initialization
            coralFunnelSensor = new DigitalInput(Constants.ArmElevatorConstants.StateConstants.kCoralFunnelSensorPort);
            coralInEndEffectorSensor = new DigitalInput(Constants.ArmElevatorConstants.StateConstants.kCoralInEndEffectorSensorPort);
        //PID & FEEDFORWARD Initialization
            elevatorPIDController = new ProfiledPIDController(
                Constants.ArmElevatorConstants.StateConstants.kPElevator,
                Constants.ArmElevatorConstants.StateConstants.kIElevator,
                Constants.ArmElevatorConstants.StateConstants.kDElevator,
                new TrapezoidProfile.Constraints(
                    Constants.ArmElevatorConstants.StateConstants.kProfileConstantsMaxSpeed,
                    Constants.ArmElevatorConstants.StateConstants.kProfileConstantsMaxAccel
                )
            );
            elevatorFeedforward = new ElevatorFeedforward(
                Constants.ArmElevatorConstants.StateConstants.kSElevator,
                Constants.ArmElevatorConstants.StateConstants.kVElevator,
                Constants.ArmElevatorConstants.StateConstants.kAElevator
            );
            armPIDController = new PIDController(
                Constants.ArmElevatorConstants.StateConstants.kPArm,
                Constants.ArmElevatorConstants.StateConstants.kIArm,
                Constants.ArmElevatorConstants.StateConstants.kDArm
            );
            armFeedforward = new SimpleMotorFeedforward(
                Constants.ArmElevatorConstants.StateConstants.kSArm,
                Constants.ArmElevatorConstants.StateConstants.kVArm,
                Constants.ArmElevatorConstants.StateConstants.kAArm
            );
        //Reset Motor Positions
            elevatorMotorA.set(0);
            elevatorMotorB.set(0);
            armMotor.set(0);
        //Initial Desired Positions
            desiredArmAngleDeg = 0.0;
            desiredElevInches = 0.0;
            currentPreset = Preset.STOW;
        //Shuffleboard Setup
            armElevatorLiveDataTab = Shuffleboard.getTab("Arm & Elevator");
                armAngleDeg = armElevatorLiveDataTab.add("Arm Angle Deg", 0).getEntry();
                elevHeightIn = armElevatorLiveDataTab.add("Elevator Height In", 0).getEntry();
                armDesiredPos = armElevatorLiveDataTab.add("Arm Desired Pos", 0).getEntry();
                elevDesiredPos = armElevatorLiveDataTab.add("Elevator Desired Pos", 0).getEntry();
                elevatorPreset = armElevatorLiveDataTab.add("Elevator Preset", "None").getEntry();
                funnelSensorEntry = armElevatorLiveDataTab.add("Funnel Sensor", false).getEntry();
                endeffectorSensorEntry = armElevatorLiveDataTab.add("End Effector Sensor", false).getEntry();
                state = armElevatorLiveDataTab.add("State", "None").getEntry(); //debugger purposes only
            armElevatorTuneDataTab = Shuffleboard.getTab("Arm & Elevator Tune Data");
                kPElevatorEntry = armElevatorTuneDataTab.add("kP Elevator", Constants.ArmElevatorConstants.StateConstants.kPElevator).getEntry();
                kIElevatorEntry = armElevatorTuneDataTab.add("kI Elevator", Constants.ArmElevatorConstants.StateConstants.kIElevator).getEntry();
                kDElevatorEntry = armElevatorTuneDataTab.add("kD Elevator", Constants.ArmElevatorConstants.StateConstants.kDElevator).getEntry();
                kSElevatorEntry = armElevatorTuneDataTab.add("kS Elevator", Constants.ArmElevatorConstants.StateConstants.kSElevator).getEntry();
                kVElevatorEntry = armElevatorTuneDataTab.add("kV Elevator", Constants.ArmElevatorConstants.StateConstants.kVElevator).getEntry();
                kAElevatorEntry = armElevatorTuneDataTab.add("kA Elevator", Constants.ArmElevatorConstants.StateConstants.kAElevator).getEntry();
                kPArmEntry = armElevatorTuneDataTab.add("kP Arm", Constants.ArmElevatorConstants.StateConstants.kPArm).getEntry();
                kIArmEntry = armElevatorTuneDataTab.add("kI Arm", Constants.ArmElevatorConstants.StateConstants.kIArm).getEntry();
                kDArmEntry = armElevatorTuneDataTab.add("kD Arm", Constants.ArmElevatorConstants.StateConstants.kDArm).getEntry();
                kSArmEntry = armElevatorTuneDataTab.add("kS Arm", Constants.ArmElevatorConstants.StateConstants.kSArm).getEntry();
                kVArmEntry = armElevatorTuneDataTab.add("kV Arm", Constants.ArmElevatorConstants.StateConstants.kVArm).getEntry();
                kAArmEntry = armElevatorTuneDataTab.add("kA Arm", Constants.ArmElevatorConstants.StateConstants.kAArm).getEntry();
    }

    //Utility Methods
        private String getPresetName(){
            switch (currentPreset){
                case STOW:
                    return "STOW";
                case FUNNEL:
                    return "FUNNEL";
                case LOADING:
                    return "LOADING";
                case L2:
                    return "L2";
                case L3:
                    return "L3";
                case L4:
                    return "L4";
                case L2Score:
                    return "L2Score";
                case L3Score:
                    return "L3Score";
                case L4Score:
                    return "L4Score";
                default:
                    return "None";
            }
        }

        //Arm Angle In Degrees
        private double getArmAngleDeg(){
            return armEncoder.getPosition() - Constants.ArmElevatorConstants.StateConstants.kArmAbsoluteEncoderOffset;
        }
        //Elevator Height In Inches
        private double getElevatorHeightInches(){
            return elevatorMotorA.getPosition().getValueAsDouble() * Constants.ArmElevatorConstants.StateConstants.kElevatorMotorRot2Inches;
        }
        //Elevator Velocity
        private double getElevatorVelocityInchesPerSec(){
            return elevatorMotorA.getVelocity().getValueAsDouble() * Constants.ArmElevatorConstants.StateConstants.kElevatorMotorRot2Inches;
        }
        //Arm Velocity
        private double getArmVelocityDegPerSec(){
            return armMotor.getVelocity().getValueAsDouble();
        }
    
        //Sensors
            private boolean getFunnelSensor(){
                return !coralFunnelSensor.get();
            }

            private boolean getEndEffectorSensor(){
                return !coralInEndEffectorSensor.get();
            }
        //Arm Tolerance Check
        private boolean isArmInTolerance(double targetAngleDeg, double toleranceDeg){
            return Math.abs(getArmAngleDeg() - targetAngleDeg) <= toleranceDeg;
        }
        //Elevator Tolerance Check
        private boolean isElevatorInTolerance(double targetHeightIn, double toleranceIn){
            return Math.abs(getElevatorHeightInches() - targetHeightIn) <= toleranceIn;
        }
        //End Effector RPM
        private double getEndEffectorRPM(){
            return endEffectorMotor.getVelocity().getValueAsDouble();
        }
        //End Effector Controls
        public void startManualIntake() {
            manualIntakeActive = true;
            autoIntakeActive = false;
            outtake = false;
        }
    
        public void startManualOuttake() {
            manualIntakeActive = true;
            autoIntakeActive = false;
            outtake = true;
        }
    
        public void slowIntake() {
            manualIntakeActive = false;
            autoIntakeActive = false;
            endEffectorMotor.set(-0.1);
            outtake = false;
        }
    
        public void stopIntake() {
            manualIntakeActive = false;
            autoIntakeActive = false;
            endEffectorMotor.set(0.0);
            outtake = false;
        }

        public Command goToStow(){
            if(currentPreset == Preset.FUNNEL){
                return Commands.runOnce(()->{
                    desiredElevInches = LevelConstants.kStowElevatorSetpoint;
                    currentPreset = Preset.STOW;
                });
            } else if(currentPreset == Preset.LOADING){
                return Commands.sequence(
                    Commands.runOnce(()->{
                        desiredElevInches = LevelConstants.kL1FunnelSetpoint;
                    }),
                    Commands.waitUntil(
                        ()-> isElevatorInTolerance(LevelConstants.kL1FunnelSetpoint, getElevatorHeightInches())
                    ),
                    Commands.runOnce(()->{
                        desiredArmAngleDeg = LevelConstants.kStowArmSetpoint;
                    }),
                    Commands.waitUntil(
                        ()-> isArmInTolerance(LevelConstants.kStowArmSetpoint, getArmAngleDeg())
                    ),
                    Commands.runOnce(()->{
                        desiredElevInches = LevelConstants.kStowElevatorSetpoint;
                    }),
                    Commands.waitUntil(
                        ()-> isElevatorInTolerance(LevelConstants.kStowElevatorSetpoint, getElevatorHeightInches())
                    ),
                    Commands.runOnce(()->{
                        currentPreset = Preset.STOW;
                    })
                );
            } else {
                return Commands.sequence(
                    Commands.runOnce(()->{
                        desiredElevInches = LevelConstants.kL1FunnelSetpoint;
                    }),
                    Commands.waitUntil(
                        ()-> isElevatorInTolerance(LevelConstants.kL1FunnelSetpoint, getElevatorHeightInches())
                    ),
                    Commands.runOnce(()->{
                        desiredArmAngleDeg = LevelConstants.kStowArmSetpoint;
                    }),
                    Commands.waitUntil(
                        ()-> isArmInTolerance(LevelConstants.kStowArmSetpoint, getArmAngleDeg())
                    ),
                    Commands.runOnce(()->{
                        desiredElevInches = LevelConstants.kStowElevatorSetpoint;
                    }),
                    Commands.waitUntil(
                        ()-> isElevatorInTolerance(LevelConstants.kStowElevatorSetpoint, getElevatorHeightInches())
                    ),
                    Commands.runOnce(()->{
                        currentPreset = Preset.STOW;
                    })
                );
            }
        }

     @Override
        public void periodic(){
            //Update Sensors
            funnelSensor = getFunnelSensor();
            endEffectorSensor = getEndEffectorSensor();

            //Shuffleboard Updates
            armAngleDeg.setDouble(getArmAngleDeg());
            elevHeightIn.setDouble(getElevatorHeightInches());
            armDesiredPos.setDouble(desiredArmAngleDeg);
            elevDesiredPos.setDouble(desiredElevInches);
            elevatorPreset.setString(getPresetName());
            funnelSensorEntry.setBoolean(funnelSensor);
            endeffectorSensorEntry.setBoolean(endEffectorSensor);
            //PID & FEEDFORWARD Updates From Shuffleboard
                //Elevator
                    //PID
                        elevatorPIDController.setP(kPElevatorEntry.getDouble(Constants.ArmElevatorConstants.StateConstants.kPElevator));
                        elevatorPIDController.setI(kIElevatorEntry.getDouble(Constants.ArmElevatorConstants.StateConstants.kIElevator));
                        elevatorPIDController.setD(kDElevatorEntry.getDouble(Constants.ArmElevatorConstants.StateConstants.kDElevator));
                    //Feedforward
                        elevatorFeedforward = new ElevatorFeedforward(
                            kSElevatorEntry.getDouble(Constants.ArmElevatorConstants.StateConstants.kSElevator),
                            kVElevatorEntry.getDouble(Constants.ArmElevatorConstants.StateConstants.kVElevator),
                            kAElevatorEntry.getDouble(Constants.ArmElevatorConstants.StateConstants.kAElevator)
                        );
                //Arm
                    //PID
                        armPIDController.setP(kPArmEntry.getDouble(Constants.ArmElevatorConstants.StateConstants.kPArm));
                        armPIDController.setI(kIArmEntry.getDouble(Constants.ArmElevatorConstants.StateConstants.kIArm));
                        armPIDController.setD(kDArmEntry.getDouble(Constants.ArmElevatorConstants.StateConstants.kDArm));
                    //Feedforward
                        armFeedforward = new SimpleMotorFeedforward(
                            kSArmEntry.getDouble(Constants.ArmElevatorConstants.StateConstants.kSArm),
                            kVArmEntry.getDouble(Constants.ArmElevatorConstants.StateConstants.kVArm),
                            kAArmEntry.getDouble(Constants.ArmElevatorConstants.StateConstants.kAArm)
                        );

            //Elevator Control
            elevatorPIDController.setGoal(Units.inchesToMeters(desiredElevInches));
                double elevOutput = elevatorPIDController.calculate(
                    Units.inchesToMeters(getElevatorHeightInches()));
                double elevFF = this.elevatorFeedforward.calculate(
                    elevatorPIDController.getSetpoint().velocity
                );
            double totalElevVolts = elevOutput + elevFF;

            //Arm Control
            armPIDController.setSetpoint(desiredArmAngleDeg);
                double armOutput = armPIDController.calculate(getArmAngleDeg());
                double armFF = armFeedforward.calculate(
                    armFeedforward.calculate(armPIDController.getSetpoint())
                );

                if (!manualElevator){
                    elevatorMotorA.setVoltage(totalElevVolts);
                    elevatorMotorB.setVoltage(totalElevVolts);
                }

                if (!manualArm){
                    armMotor.setVoltage(armOutput + armFF);
                }
        }
}
