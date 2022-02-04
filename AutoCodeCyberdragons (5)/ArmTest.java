package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.logging.Level;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "ArmTest", group="Autonomous")
public class ArmTest extends LinearOpMode {
    private DcMotor ARM;
    private int ArmPos = 0;
    int[] ArmLevelTicks = {100, 300, 500, 700};  // declares an array of integers for arm level heights
    private double Holding_Speed = 0.0;
    private double Down_Speed = 0.375;
    private double Up_Speed = 1.0;

    @Override
    public void runOpMode() {
        ARM = hardwareMap.get(DcMotor.class, "Arm");
        ARM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ARM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ARM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        waitForStart();
        rotTo(ArmLevelTicks[0], Down_Speed);
        sleep(1000);
        
        rotTo(ArmLevelTicks[1], Up_Speed);
        sleep(3000);
        
        rotTo(ArmLevelTicks[2], Up_Speed);
        sleep(3000);
        
        rotTo(ArmLevelTicks[3], Up_Speed);
        sleep(3000);
        
        rotTo(ArmLevelTicks[0], Down_Speed);
        sleep(3000);
        // rotTo(mid, Up);
        // rotTo(top, 0.1);
        // rotTo(btm, 0.1);
    }

    private void rotTo(int ticks, double speed) {
        ARM.setTargetPosition(ticks);
        ARM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ARM.setPower(speed);

        while (opModeIsActive() && ARM.isBusy()) {
            telemetry.addData("EncoderArm", ARM.getCurrentPosition() + "  busy=" + ARM.isBusy());
            telemetry.update();
            idle();
        }

        ARM.setPower(Holding_Speed);
        sleep(100);
        ArmPos = ARM.getTargetPosition();
        //   sleep(5000);
        
    }
}
