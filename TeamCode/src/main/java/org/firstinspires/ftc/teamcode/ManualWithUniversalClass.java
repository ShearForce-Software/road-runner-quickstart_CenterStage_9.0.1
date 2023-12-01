package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.UniversalControlClass;
@TeleOp(name = "1 Manual Control")
public class ManualWithUniversalClass extends LinearOpMode {
    UniversalControlClass theRobot = new UniversalControlClass(true, true, this);
    static final double SCALE = 0.001;
    public void runOpMode() {
        theRobot.Init(this.hardwareMap);
        theRobot.ManualStartPos();
        theRobot.ShowSlideTelemetry();
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            theRobot.driveControlsFieldCentric();
            theRobot.PickupRoutine();
            //theRobot.CheckForSlideBottom();
            if (gamepad2.right_bumper) { // intake in
                theRobot.EnableAutoIntake();
            }
            if(gamepad2.dpad_left){
                theRobot.ServoOuttake();
                theRobot.DisableAutoIntake();
            }
            else if(gamepad2.dpad_right) {
                theRobot.ServoStop();
                theRobot.DisableAutoIntake();
            }
            if (gamepad2.left_bumper){ // release pixels
                theRobot.ReleaseRight();
                theRobot.ReleaseLeft();
            }
            if(gamepad2.y){
                theRobot.ServoStop();
                theRobot.SlidesHigh();
                theRobot.DeliverPixelToBoardPos();
            }
            if(gamepad2.x){
                theRobot.ServoStop();
                theRobot.SlidesMedium();
                theRobot.DeliverPixelToBoardPos();
            }
            if(gamepad2.a){
                theRobot.ServoStop();
                theRobot.SlidesLow();
                theRobot.DeliverPixelToBoardPos();
            }
            if(gamepad2.b){
                theRobot.ResetArm();
            }
            if((gamepad2.left_trigger!=0)&&(gamepad2.right_trigger!=0)){
                theRobot.LaunchAirplane();
            }
            theRobot.ShowSlideTelemetry();
            telemetry.update();
        }
    }
}
