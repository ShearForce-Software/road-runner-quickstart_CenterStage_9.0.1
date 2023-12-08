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



        try {
            while (opModeIsActive()) {
                theRobot.driveControlsFieldCentric();
                theRobot.PickupRoutine();
                //theRobot.CheckForSlideBottom();
                if (gamepad2.right_bumper) { // intake in
                    theRobot.EnableAutoIntake();
                }
                if (gamepad2.dpad_left) {
                    theRobot.ServoOuttake();
                    theRobot.DisableAutoIntake();
                } else if (gamepad2.dpad_right) {
                    theRobot.ServoStop();
                    theRobot.DisableAutoIntake();
                }
                if (gamepad2.left_bumper) { // release pixels
                    theRobot.ReleaseRight();
                    theRobot.ReleaseLeft();
                }
                if (gamepad2.y && !gamepad2.back) {
                    theRobot.ServoStop();
                    theRobot.SlidesHigh();
                    theRobot.SpecialSleep(500);
                    theRobot.DeliverPixelToBoardPos();
                }
                if (gamepad2.x) {
                    theRobot.ServoStop();
                    theRobot.SlidesMedium();
                    theRobot.SpecialSleep(500);
                    theRobot.DeliverPixelToBoardPos();
                }
                if (gamepad2.a && !gamepad2.start) {
                    theRobot.ServoStop();
                    theRobot.SlidesLow();
                    theRobot.SpecialSleep(500);
                    theRobot.DeliverPixelToBoardPos();
                }
                if (gamepad2.b && !gamepad2.start) {
                    theRobot.ResetArm();
                }
                if (gamepad1.a && !gamepad1.start) {
                    theRobot.LaunchAirplane();
                }

            if (gamepad2.left_stick_y < -0.10)
            {
                theRobot.SetSlidePower(-.5);
            }
            else if (gamepad2.left_stick_y > 0.1)
            {
                theRobot.SetSlidePower(.5);
            }
            else if (gamepad2.dpad_up)
            {
                theRobot.SetSlidePower(0);
            }

            if (gamepad2.right_stick_x > 0.1)
            {
                theRobot.ArmWrist(theRobot.getWristPosition() + 0.01);
                theRobot.SpecialSleep(150);
            }
            else if (gamepad2.right_stick_x < -0.1)
            {
                theRobot.ArmWrist(theRobot.getWristPosition() - 0.01);
                theRobot.SpecialSleep(150);
            }

                if (gamepad1.left_trigger != 0) {
                    theRobot.SetScissorLiftPower(gamepad1.left_trigger);
                } else if (gamepad1.right_trigger != 0) {
                    theRobot.SetScissorLiftPower(-gamepad1.right_trigger);
                } else {
                    theRobot.SetScissorLiftPower(0);
                }

                // special combo (left tiny button and top right button together)
                if (gamepad2.back && gamepad2.y) {
                    theRobot.ResetWristGrabPixelPos();
                    theRobot.SpecialSleep(150);
                }

                theRobot.ShowSlideTelemetry();
                telemetry.update();
            } // end while (opModeIsActive())

            // Stop must have been pressed to get here, make sure the continuous rotation servos are stopped
            theRobot.ServoStop();
            sleep(150);

        } catch (Exception e) {
            // something went wrong and has crashed
            // Make sure the continuous rotation servos are stopped
            theRobot.ServoStop();
            sleep(150);

            // throw the exception higher for other handlers to run
            throw e;
        }
    }
}
