package com.cyberdragons;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="BobAI", group="Linear Opmode")

public class BobAI extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor WHEEL_FR = null;
    private DcMotor WHEEL_BR = null;
    private DcMotor WHEEL_FL = null;
    private DcMotor WHEEL_BL = null;
    private DcMotor TTS_Motor = null;
    private DcMotor arm = null;
    private DcMotor Flapper = null;
    
    private DistanceSensor TOF_Front = null;
    private DistanceSensor TOF_Left = null;
    private DistanceSensor TOF_Right = null;

    private DcMotor IntakeArm = null;
    private Servo Lid = null; // PreLoaded Block Dropper
    private Servo Eyes = null;

    final double NormalDriveSpeed = 0.7; // Max Throttle for the Drive Motors Normally - Value Between 0 and 1
    final double NormalCrabSpeed = 0.8; // Max Throttle for the Drive Motors Normally - Value Between 0 and 1
    final double NormalTurnSpeed = 0.6; // Max Throttle for the Drive Motors Normally - Value Between 0 and 1
    final double MicroDriveSpeed = 0.1725; // Max Throttle for the Drive Motors When In Micro - Value Between 0 and 1
    final double MicroCrabSpeed = 0.3; // Max Throttle for the Drive Motors When In Micro - Value Between 0 and 1
    final double MicroTurnSpeed = 0.3; // Max Throttle for the Drive Motors When In Micro - Value Between 0 and 1
    final double MidDriveSpeed = 0.35; // Max Throttle for the Drive Motors When In Micro - Value Between 0 and 1
    final double MidCrabSpeed = 0.4; // Max Throttle for the Drive Motors When In Micro - Value Between 0 and 1
    final double MidTurnSpeed = 0.4; // Max Throttle for the Drive Motors When In Micro - Value Between 0 and 1
    final double TTSpeed = 1; // Speed for the Turn Table Drive Motors - Value Between 0 and 1
    final double FlapperSpeed = 1;
    final double IntakeSpeed = 1;
    
    double DriveSpeed;
    double CrabSpeed;
    double TurnSpeed;
    
    
    
    
    double FrontLongDist = 50;
    double FrontMidDist = 25;
    double FrontShortDist = 15;
    
    double SideLongDist = 50;
    double SideMidDist = 25;
    double SideShortDist = 15;
    
    boolean FrontLongClear = false;
    boolean FrontMidClear = false;
    boolean FrontShortClear = false;
    
    boolean LeftLongClear = false;
    boolean LeftMidClear = false;
    boolean LeftShortClear = false;
    
    boolean RightLongClear = false;
    boolean RightMidClear = false;
    boolean RightShortClear = false;
    
    
    String Bob = "HAPPY";
    
    
    double Frontcm;
    double Leftcm;
    double Rightcm;
    
    

    final double ArmSpeed = 0.5; //1
    final double MicroArmSpeed = 0.2; //0.4
    boolean armReachedTarget = true;
    double ArmTicksPerRot;
    // [0]: ticks for ground level, [1]: ticks for level 1, [2]: ticks for level 2, [3]: ticks for level 3
    // [4]: ticks for level 3 (flipped), [5]: ticks for level 2 (flipped), [6]: ticks for level 1 (flipped), [7]: ticks for ground level (flipped)
    int[] ArmLevelTicks;

    boolean armOpen = false;
    boolean intakeUp = true;
    boolean boxUp = false;
    boolean IntakeArmGoUp = false;
    boolean ArmDown = true;
    
    boolean xButtonDown1;
    boolean yButtonDown1;
    boolean aButtonDown1;
    boolean bButtonDown1;
    boolean xButtonDown2;
    boolean yButtonDown2;
    boolean aButtonDown2;
    boolean bButtonDown2;
    boolean rightBumperDown;
    

  

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        WHEEL_FR = hardwareMap.get(DcMotor.class, "wheelfr"); // Front Right
        WHEEL_BR = hardwareMap.get(DcMotor.class, "wheelbr"); // Back Right
        WHEEL_FL = hardwareMap.get(DcMotor.class, "wheelfl"); // Front Left
        WHEEL_BL = hardwareMap.get(DcMotor.class, "wheelbl"); // Back Left
        TTS_Motor = hardwareMap.get(DcMotor.class, "duck");   // Motor for Duck Turn Table Spinner
        Flapper = hardwareMap.get(DcMotor.class, "Flapper");
        IntakeArm = hardwareMap.get(DcMotor.class, "IntakeArm"); //Arm with flapper
        Lid = hardwareMap.get(Servo.class, "Lid");         // Severo to drop pre loaded block drop
        Eyes = hardwareMap.get(Servo.class, "Eyes");
        TOF_Left = hardwareMap.get(DistanceSensor.class, "TOF_Left");   // Time of Flight Sensor mounted on the left of the robot
        TOF_Right = hardwareMap.get(DistanceSensor.class, "TOF_Right");   // Time of Flight Sensor mounted on the right of the robot
        TOF_Front = hardwareMap.get(DistanceSensor.class, "TOF_Front");// Time of Flight Sensor mounted on the front of the robot


        TTS_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        Flapper.setDirection(DcMotorSimple.Direction.FORWARD);

        arm = hardwareMap.dcMotor.get("Arm");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        IntakeArm.setDirection(DcMotorSimple.Direction.FORWARD);
        IntakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setArmMotorType(ArmMotorType.Rev20);


        moveArm(150,0.25);
        MoveIntake(-100, 0.1);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // Set the default motor directions, Left and Right Wheel Motors are Flipped in Their Mounting, so Their Default Dirrections are Opposites
            WHEEL_FL.setDirection(DcMotorSimple.Direction.FORWARD);
            WHEEL_FR.setDirection(DcMotorSimple.Direction.REVERSE);
            WHEEL_BL.setDirection(DcMotorSimple.Direction.FORWARD);
            WHEEL_BR.setDirection(DcMotorSimple.Direction.REVERSE);

            /*
            if (gamepad1.right_bumper) {
                DriveSpeed = MicroDriveSpeed;
                CrabSpeed = MicroCrabSpeed;
                TurnSpeed = MicroTurnSpeed;
            }
            else if (gamepad1.left_bumper) {
            */
                DriveSpeed = MidDriveSpeed;
                CrabSpeed = MidCrabSpeed;
                TurnSpeed = MidTurnSpeed;
                /*
            }
            else {
                DriveSpeed = NormalDriveSpeed;
                CrabSpeed = NormalCrabSpeed;
                TurnSpeed = NormalTurnSpeed;
            }
            */


          

            //Run the Wheels bases on the Joystick Positions on GamePad1

                //Drive version 1 (Left Joystick controls forward/backward and rotation, Right Joystick controls crabbing)
                /*
                WHEEL_FR.setPower(+(DriveSpeed * gamepad1.left_stick_y + DriveSpeed * gamepad1.left_stick_x + CrabSpeed * gamepad1.right_stick_x));
                WHEEL_BR.setPower(+(DriveSpeed * gamepad1.left_stick_y + DriveSpeed * gamepad1.left_stick_x - CrabSpeed * gamepad1.right_stick_x));
                WHEEL_FL.setPower(+(DriveSpeed * gamepad1.left_stick_y - DriveSpeed * gamepad1.left_stick_x - CrabSpeed * gamepad1.right_stick_x));
                WHEEL_BL.setPower(+(DriveSpeed * gamepad1.left_stick_y - DriveSpeed * gamepad1.left_stick_x + CrabSpeed * gamepad1.right_stick_x));
                */

                //Drive version 2 (Left Joystick controls forward/backward and crabbing, Right Joystick controls rotation)
                /*
                WHEEL_FR.setPower(+(DriveSpeed * gamepad1.left_stick_y + TurnSpeed * gamepad1.right_stick_x + CrabSpeed * gamepad1.left_stick_x));
                WHEEL_BR.setPower(+(DriveSpeed * gamepad1.left_stick_y + TurnSpeed * gamepad1.right_stick_x - CrabSpeed * gamepad1.left_stick_x));
                WHEEL_FL.setPower(+(DriveSpeed * gamepad1.left_stick_y - TurnSpeed * gamepad1.right_stick_x - CrabSpeed * gamepad1.left_stick_x));
                WHEEL_BL.setPower(+(DriveSpeed * gamepad1.left_stick_y - TurnSpeed * gamepad1.right_stick_x + CrabSpeed * gamepad1.left_stick_x));
                */
                
                
                //Drive version 3 (Left Joystick (flipped) controls forward/backward and crabbing, Right Joystick controls rotation)
                /*
                WHEEL_FR.setPower(+(DriveSpeed * -gamepad1.left_stick_y + TurnSpeed * gamepad1.right_stick_x + CrabSpeed * gamepad1.left_stick_x));
                WHEEL_BR.setPower(+(DriveSpeed * -gamepad1.left_stick_y + TurnSpeed * gamepad1.right_stick_x - CrabSpeed * gamepad1.left_stick_x));
                WHEEL_FL.setPower(+(DriveSpeed * -gamepad1.left_stick_y - TurnSpeed * gamepad1.right_stick_x - CrabSpeed * gamepad1.left_stick_x));
                WHEEL_BL.setPower(+(DriveSpeed * -gamepad1.left_stick_y - TurnSpeed * gamepad1.right_stick_x + CrabSpeed * gamepad1.left_stick_x));
                */
                
                
                
                
                
                
                
            Frontcm = TOF_Front.getDistance(DistanceUnit.CM);
            Leftcm = TOF_Left.getDistance(DistanceUnit.CM);
            Rightcm = TOF_Right.getDistance(DistanceUnit.CM);
                
            telemetry.addData("Front", Frontcm);
            telemetry.addData("Left", Leftcm);
            telemetry.addData("Right", Rightcm);
            telemetry.update();
             
             
            
            if(Frontcm > FrontLongDist){
                FrontLongClear = true;
            }else{
                FrontLongClear = false;
            }
            if(Frontcm > FrontMidDist){
                FrontMidClear = true;
            }else{
                FrontMidClear = false;
            }
            if(Frontcm > FrontShortDist){
                FrontShortClear = true;
            }else{
                FrontShortClear = false;
            }
            
            
            
            
            if(Leftcm > SideLongDist){
                LeftLongClear = true;
            }else{
                LeftLongClear = false;
            }
            if(Leftcm > SideMidDist){
                LeftMidClear = true;
            }else{
                LeftMidClear = false;
            }
            if(Leftcm > SideShortDist){
                LeftShortClear = true;
            }else{
                LeftShortClear = false;
            }
            
            
            
            if(Rightcm > SideLongDist){
                RightLongClear = true;
            }else{
                RightLongClear = false;
            }
            if(Rightcm > SideMidDist){
                RightMidClear = true;
            }else{
                RightMidClear = false;
            }
            if(Rightcm > SideShortDist){
                RightShortClear = true;
            }else{
                RightShortClear = false;
            }
        
            
             
                
            if(FrontLongClear){
                WHEEL_FR.setPower(-DriveSpeed);
                WHEEL_BR.setPower(-DriveSpeed);
                WHEEL_FL.setPower(-DriveSpeed);
                WHEEL_BL.setPower(-DriveSpeed);
            }else if(LeftLongClear || RightLongClear){
             if(Leftcm > Rightcm){
                    WHEEL_FR.setPower(-TurnSpeed);
                    WHEEL_BR.setPower(-TurnSpeed);
                    WHEEL_FL.setPower(+TurnSpeed);
                    WHEEL_BL.setPower(+TurnSpeed);
                }else if(Rightcm > Leftcm){
                    WHEEL_FR.setPower(+TurnSpeed);
                    WHEEL_BR.setPower(+TurnSpeed);
                    WHEEL_FL.setPower(-TurnSpeed);
                    WHEEL_BL.setPower(-TurnSpeed);
                }
            }else if(LeftShortClear || RightShortClear){
                if(Leftcm > Rightcm){
                    WHEEL_FR.setPower(+CrabSpeed);
                    WHEEL_BR.setPower(-CrabSpeed);
                    WHEEL_FL.setPower(-CrabSpeed);
                    WHEEL_BL.setPower(+CrabSpeed);
                }else if(Rightcm > Leftcm){
                    WHEEL_FR.setPower(+CrabSpeed);
                    WHEEL_BR.setPower(-CrabSpeed);
                    WHEEL_FL.setPower(-CrabSpeed);
                    WHEEL_BL.setPower(+CrabSpeed);
                }
            }
            
            else if(!FrontLongClear && !LeftLongClear && !RightLongClear){
                WHEEL_FR.setPower(0);
                WHEEL_BR.setPower(0);
                WHEEL_FL.setPower(0);
                WHEEL_BL.setPower(0);
            }
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                

          
             



            //Bobs personality
            
//#################################################################################//            
            
            //ADVANCED AI (DO NOT TOUCH)
    
                    
    
            if(Bob == "HAPPY"){
                Eyes.setPosition(0.2);
                telemetry.addData("Bob", "Happy");
            }else if(Bob == "SAD"){
                Eyes.setPosition(0.5);
                telemetry.addData("Bob", "Sad");
            }else if(Bob == "WORRIED"){
                Eyes.setPosition(0.3);
                telemetry.addData("Bob", "Worried");
            }else if(Bob == "MAD"){
                Eyes.setPosition(0.1);
                telemetry.addData("Bob", "Mad");
            }else if(Bob == "ANGRY"){
                Eyes.setPosition(0.05);
                telemetry.addData("Bob", "Angry");
            }



//#################################################################################//         




            if(arm.getCurrentPosition()<200){
                ArmDown = true;
            }else{
                ArmDown = false;
            }



            
            
            

        












            
        }
    }

    enum ArmMotorType {
        Andy20,
        Rev20,
        Rev40,
        Rev60,
        Tetrix,
    }

    private void setArmMotorType(ArmMotorType type) {
        switch (type) {
            case Andy20:
                ArmTicksPerRot = 537.6;
                ArmLevelTicks = new int[] {100, 250, 500, 750};
                break;
            case Rev20:
                ArmTicksPerRot = 560;
                ArmLevelTicks = new int[]{0, 1000, 900, 760, 200};
                break;
            case Rev60: // same settings as 'REV20'
                ArmTicksPerRot = 1680.0;
                ArmLevelTicks = new int[] {100, 250, 500, 750};
                break;
            case Rev40:
                ArmTicksPerRot = 1120.0;
                ArmLevelTicks = new int[] {100, 250, 500, 750};
                break;
            case Tetrix:
                ArmTicksPerRot = 1440;
                //ArmLevelTicks = new int[]{-10, -300, -550, -850};
                ArmLevelTicks = new int[]{0, 2800, 2550, 2200, 2000};
                break;
        }
    }

    private void moveArm(int ticks, double speed) {
        arm.setTargetPosition(ticks);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        arm.setPower(speed);
        
        
        armReachedTarget = false;
    }
    
    
    private void MoveIntake(int ticks, double speed) {
        IntakeArm.setTargetPosition(ticks);
        IntakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        IntakeArm.setPower(speed);
        
        
        //armReachedTarget = false;
    }
}
