

package org.firstinspires.ftc.robotcontroller.external.samples;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import java.util.logging.Level;
import java.util.*;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;




@Autonomous(name = "PickUpTest", group="Autonomous")
public class PickUpTest extends LinearOpMode { //Pick up block


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor WHEEL_FL;
    private DcMotor WHEEL_FR;
    private DcMotor WHEEL_BL;
    private DcMotor WHEEL_BR;
    private DcMotor TTS_Motor;
    private DcMotor ARM;
    private DcMotor Flapper;
    private DistanceSensor TOF_Left; // Time Of Flight Left
    private DistanceSensor TOF_Right; //Time of Flight right
    private DistanceSensor TOF_Front;
    private Servo Lid;           //lid
    private DcMotor IntakeArm;
    private Servo Eyes;
    private DigitalChannel Linebreak;

    String MOTORTYPE = "REV20"; // "REV20", "REV40", "REV60", "TETRIX"
    String ARMMOTOR = "REV20"; // "REV20", "REV40", "REV60", "TETRIX"

    int[] ArmLevelTicks = {0, 0, 0, 0};  // declares an array of integers for arm level heights
    // ArmLevelTicks[Level] -> Ticks to move the arm to
    // ArmLevelTicks[0] = Arm Rest Position after Initialization
    // ArmLevelTicks[Level] = Birdbath Levels

    // That is which bar code corresponds to which birdbath level, it switches depending on which side ofthe bird bath you are on
    int StartPosition = 3;         // 1 = Red Closest to duck spinner, 2 = Red Closest to Warehouse; 3 = Blue Closest to duck spinner, 4 = Blue Closest to Warehouse

    double WheelTicksPerRot; // DEFAULT to ANDYMARK
    double WheelDiameterMM;         // The Diameter of the drive wheels in mm
    double DistPerRot;              // Distance traveled, either forward or backward, per 1 complete wheel rotation
    double TickPerDegree;           // The number of motor ticks to rotate (tank-like turn) the robot, either clockwise (CW) or counter-clockwise (CCW)
    double DistCrabPerRot;          // Distance crabbed, either left or right, per 1 complete wheel rotation

    int Step = -1;            // Variable to track which step of the of autonomous to perform
    int[] BarCodeLevels = {1, 2, 3}; // An int array for which barcode level corresponds to which level on the birdbath
    int Level = BarCodeLevels[0];   // Need to update when updating start position that is if the BarCodeLevels is reversed based on starting position
    //int StartLevel = 2;
    boolean ObjDetected = false;    // Boolean, for if there Special Object has been detected
    double BC_Dist = 67.5;          // Bar Code Distance
    double TOFcm = 0.0;             // Time of Flight Measurement in cm
    
    
    double Frontcm;
    double Leftcm;
    double Rightcm;

    int WHEEL_FR_pos = 0;           // Front Right Wheel Position in Encoder Steps
    int WHEEL_FL_pos = 0;           // Front Left Wheel Position in Encoder Steps
    int WHEEL_BR_pos = 0;           // Back Right Wheel Position in Encoder Steps
    int WHEEL_BL_pos = 0;           // Back Left Wheel Position in Encoder Steps

    double ArmTicksPerRot; // DEFAULT to ANDYMARK
    double ArmDownSpeed = 0.35;      // Speed to move the arm down 0.0<->1.0
    double ArmUpSpeed = 1.00;     // Speed to move the arm down 0.0<->1.0
    double ArmHoldSpeed = 0.1;      // Holding Current For the Arm
    
    
    int IntakeUp = 0;
    int IntakeDown = -1000;
    boolean LidOpen = false;
    double IntakeSpeed = 1;
    
    double ARM_pos;
    double closed = 0.0, open = 0.35;
    long ToTheSide;
    int TurnAmount;
    
    
    
    boolean intakeUp = true;
    boolean intakeSet = false;
    boolean boxUp = false;
    boolean IntakeArmGoUp = false;
    boolean IntakeSetUp = false;
    boolean ArmDown = true;
    boolean GotBlock = false;
    boolean BlockInBox = true;
    boolean PickUpBlock = false;
    
    
    
    final double BoxOpenPos = 0.8;
    final double BoxFullOpenPos = 0.9;
    final double BoxClosedPos = 0.2;
    final double BoxOpenTopLevel = 0.8;
    final double BoxOpenMiddleLevel = 0.8;
    final double BoxOpenBottomLevel = 0.8;
    
    
    
    
    
    
    

    @Override
    public void runOpMode() {

        switch (MOTORTYPE) {
            case "ANDY20":
                WheelTicksPerRot = 537.6;
                WheelDiameterMM = 103.5;
                TickPerDegree = 8.769; //
                DistCrabPerRot = 267.5; //mm
                break;
            case "REV20":
                WheelTicksPerRot = 430.77;
                WheelDiameterMM = 105.0;
                TickPerDegree = 5.567; //6.69 5.9733
                DistCrabPerRot = 260; //still to be empiracally derived
                break;
            case "REV40":
                WheelTicksPerRot = 1120.0;
                WheelDiameterMM = 101.0;
                TickPerDegree = 1.46752 / 90;
                DistCrabPerRot = 10; //still to be empiracally derived
                break;
            case "REV60":
                WheelTicksPerRot = 1680.0;
                WheelDiameterMM = 101.0;
                TickPerDegree = 1.46752 / 90;
                DistCrabPerRot = 10; //still to be empiracally derived
                break;
            case "TETRIX":
                WheelTicksPerRot = 1440;
                WheelDiameterMM = 101.0;
                TickPerDegree = 1.46752 / 90;
                DistCrabPerRot = 10; //still to be empiracally derived
                break;
            default:
                telemetry.addData("Wheel Motors not specified", "defaulting to ANDY20");
                telemetry.update();

                WheelTicksPerRot = 537.6;
                WheelDiameterMM = 101.0;
                TickPerDegree = 1.46752 / 90;
                DistCrabPerRot = 10; //still to be empirically derived
                break;
        } // switch(MOTORTYPE)

        switch (ARMMOTOR) {

            case "ANDY20":
                ArmTicksPerRot = 537.6;
                ArmLevelTicks = new int[]{100, 250, 500, 750}; // ArmLevelTicks[0] = ticks for level 1, ArmLevelTicks[1] = ticks for level 2, ...
                break;
            case "REV20":
                ArmTicksPerRot = 560;
                ArmLevelTicks = new int[]{0, 1000, 880, 750, 200}; // ArmLevelTicks[0] = ticks for down, ArmLevelTicks[1] = ticks for level 1, ...
                break;
            case "REV40":
                ArmTicksPerRot = 1120.0;
                ArmLevelTicks = new int[]{100, 250, 500, 750}; // ArmLevelTicks[0] = ticks for level 1, ArmLevelTicks[1] = ticks for level 2, ...
                break;
            case "REV60":
                ArmTicksPerRot = 1680.0;
                ArmLevelTicks = new int[]{100, 250, 500, 750}; // ArmLevelTicks[0] = ticks for level 1, ArmLevelTicks[1] = ticks for level 2, ...
                break;
            case "TETRIX":
                ArmTicksPerRot = 1440;
                ArmLevelTicks = new int[]{-50, -250, -500, -770}; // ArmLevelTicks[0] = ticks for level 1, ArmLevelTicks[1] = ticks for level 2, ...
                break;
            default:
                telemetry.addData("Arm Motor not specified", "defaulting to ANDY20");
                telemetry.update();

                ArmTicksPerRot = 537.6;
                break;
        } // switch(ARMMOTOR)



        DistPerRot = Math.PI * WheelDiameterMM; // Distance traveled per full wheel rotation in mm


        WHEEL_FL = hardwareMap.get(DcMotor.class, "wheelfl");           // Front Left
        WHEEL_FR = hardwareMap.get(DcMotor.class, "wheelfr");           // Front Right
        WHEEL_BL = hardwareMap.get(DcMotor.class, "wheelbl");           // Back Left
        WHEEL_BR = hardwareMap.get(DcMotor.class, "wheelbr");           // Back Right
        TTS_Motor = hardwareMap.get(DcMotor.class, "duck");             // Motor for Duck Turn Table Spinner
        ARM = hardwareMap.get(DcMotor.class, "Arm");                // Motor for raising and lowering the arm
        TOF_Left = hardwareMap.get(DistanceSensor.class, "TOF_Left");   // Time of Flight Sensor mounted on the left of the robot
        TOF_Right = hardwareMap.get(DistanceSensor.class, "TOF_Right");   // Time of Flight Sensor mounted on the right of the robot
        TOF_Front = hardwareMap.get(DistanceSensor.class, "TOF_Frnt");
        Lid = hardwareMap.get(Servo.class, "Lid");              // lid Servo
        Flapper = hardwareMap.get(DcMotor.class, "Flapper");
        IntakeArm = hardwareMap.get(DcMotor.class, "IntakeArm"); //Arm with flapper
        Eyes = hardwareMap.get(Servo.class, "Eyes");
        Linebreak = hardwareMap.get(DigitalChannel.class, "Linebreak");


        WHEEL_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WHEEL_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WHEEL_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WHEEL_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reseting encoder value
        WHEEL_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WHEEL_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WHEEL_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WHEEL_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TTS_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Still need to wire this up
        ARM.setDirection(DcMotorSimple.Direction.REVERSE);
        ARM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lid.setPosition(closed);
        

        
        IntakeArm.setDirection(DcMotorSimple.Direction.FORWARD);
        IntakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        Flapper.setDirection(DcMotorSimple.Direction.FORWARD);

        WHEEL_FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WHEEL_FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WHEEL_BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WHEEL_BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TTS_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ARM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



            WHEEL_FR.setDirection(DcMotorSimple.Direction.REVERSE);
            WHEEL_BR.setDirection(DcMotorSimple.Direction.REVERSE);

        ARM.setDirection(DcMotorSimple.Direction.REVERSE); 


        Frontcm = TOF_Front.getDistance(DistanceUnit.CM);
         if(Frontcm < 15){
             MoveIntake(950, 0.6);
             IntakeDown = 0;
             IntakeUp = 950;
             telemetry.addData("WARNING", "Robot did not start within 18 inches!");
         }else{
             IntakeDown = -950;
             IntakeUp = 0;
         }

        // *******************************************************************************
        // Step == -1 //Initialization
        // *******************************************************************************

        Lid.setPosition(BoxOpenPos);
        LidOpen = true;





        // *******************************************************************************


        // *******************************************************************************
        // Waiting for "Play" button to be pressed on the driver hub
        // *******************************************************************************

        waitForStart();


        while(opModeIsActive()){
            BlockInBox = Linebreak.getState();
            GotBlock = !BlockInBox;
        }

        while(PickUpBlock){
            
        }
        
        MoveIntake(IntakeDown, IntakeSpeed);
        intakeUp = false;
        
        drive(-200, 0.1);
        
        
        
        
      
        
        
        
        
        
        
        
        
    } // End runOpMode

    //******************************
    //********** SubFXNs ***********
    //******************************

    private void turn(double angle, double speed) {

        // int LeftRot = (int) Math.round( angle * TickPerDegree * TicksPerRot);
        // int RightRot = (int) Math.round( -angle * TickPerDegree * TicksPerRot);
        int LeftRot = (int) Math.round(angle * TickPerDegree);
        int RightRot = (int) Math.round(-angle * TickPerDegree);

        WHEEL_FR.setTargetPosition(RightRot + WHEEL_FR_pos);
        WHEEL_FL.setTargetPosition(LeftRot + WHEEL_FL_pos);
        WHEEL_BR.setTargetPosition(RightRot + WHEEL_BR_pos);
        WHEEL_BL.setTargetPosition(LeftRot + WHEEL_BL_pos);

        RunMotors(speed);

    } // End turn


    private void crab(double distance, double speed) {

        int ROT = (int) Math.round((distance / DistCrabPerRot) * WheelTicksPerRot);

        WHEEL_FR.setTargetPosition(-ROT + WHEEL_FR_pos);
        WHEEL_FL.setTargetPosition(ROT + WHEEL_FL_pos);
        WHEEL_BR.setTargetPosition(ROT + WHEEL_BR_pos);
        WHEEL_BL.setTargetPosition(-ROT + WHEEL_BL_pos);

        RunMotors(speed);

    } // End crab


    private void drive(double distance, double speed) {

        int ROT = (int) Math.round((distance / DistPerRot) * WheelTicksPerRot);

        WHEEL_FR.setTargetPosition(ROT + WHEEL_FR_pos);
        WHEEL_FL.setTargetPosition(ROT + WHEEL_FL_pos);
        WHEEL_BR.setTargetPosition(ROT + WHEEL_BR_pos);
        WHEEL_BL.setTargetPosition(ROT + WHEEL_BL_pos);

        RunMotors(speed);

    } // End drive


    private void RunMotors(double speed) {

        WHEEL_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        WHEEL_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        WHEEL_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        WHEEL_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        WHEEL_FL.setPower(speed);
        WHEEL_FR.setPower(speed);
        WHEEL_BL.setPower(speed);
        WHEEL_BR.setPower(speed);

        while (opModeIsActive() && isWheelsMoving()) {
            DoStuffWhileMoving();
        }

        WHEEL_FL.setPower(0.0);
        WHEEL_FR.setPower(0.0);
        WHEEL_BL.setPower(0.0);
        WHEEL_BR.setPower(0.0);

        sleep(100); // Allow for wheels to settle before getting final Position

        WHEEL_FR_pos = WHEEL_FR.getCurrentPosition();
        WHEEL_FL_pos = WHEEL_FL.getCurrentPosition();
        WHEEL_BR_pos = WHEEL_BR.getCurrentPosition();
        WHEEL_BL_pos = WHEEL_BL.getCurrentPosition();

    } // End RunMotors


    // FXN to clean up the code
    private boolean isWheelsMoving() {
        return WHEEL_FL.isBusy() && WHEEL_FR.isBusy() && WHEEL_BL.isBusy() && WHEEL_BR.isBusy();
    } // End wheelIsMoving


    private void RunArm(int ArmTicks, double ArmSpeed) {

        ARM.setTargetPosition(ArmTicks);
        ARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ARM.setPower(ArmSpeed);

        while (opModeIsActive() && ARM.isBusy()) {idle();} //
        
        ARM.setPower(ArmHoldSpeed);
        sleep(100); // Allow for wheels to settle before getting final Position
        ARM_pos = ARM.getCurrentPosition();

    } // End RunArm
    
    private void MoveIntake(int ticks, double speed) {
        IntakeArm.setTargetPosition(ticks);
        IntakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        IntakeArm.setPower(speed);


    }


    private void DoStuffWhileMoving() {


        switch (Step) {
            case 0:
                break;
            case 1: 
                
                break;
            case 2:
                break;
            default:

                break;
        } // end switch case

    } // end DoStuffWhileMoving


} // End DriveTest
