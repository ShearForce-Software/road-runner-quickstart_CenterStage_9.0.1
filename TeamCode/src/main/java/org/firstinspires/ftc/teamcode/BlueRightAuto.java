package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Blue Right", preselectTeleOp="1 Manual Control")
public class BlueRightAuto extends LinearOpMode {
    UniversalControlClass control = new UniversalControlClass(true, false,this);
    private HuskyLens huskyLens;
    private final int READ_PERIOD = 1;
    int leftRightSpikeBound = 150;
    int autoPosition = 0;
    public double pixelDeliverFirstPos = 14.5;
    public void runOpMode(){
        Pose2d startPose = new Pose2d(-38.5,62.5,Math.toRadians(270));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        control.Init(hardwareMap);
        control.HuskyLensInit();
        control.AutoStartPos();

        telemetry.update();

        control.WebcamInit(hardwareMap);
        telemetry.update();

        while(!isStarted()){
            control.DetectTeamArtBlue();
            telemetry.update();
        }
        waitForStart();

        // lock the pixels
        control.GrabPixels();

        // drive to the middle floor delivery position
        Actions.runBlocking(
            drive.actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-38.5, 12.5, Math.toRadians(270)), Math.toRadians(270))
                .build());

        // Drop the LEFT pixel (put PURPLE on LEFT, YELLOW on RIGHT) on the line
        control.DropOnLine();
        // put the arm back in a safe to travel position
        control.SafeStow();

        // drive to the backboard area
        Actions.runBlocking(
            drive.actionBuilder(new Pose2d(-38.5, 12.5, Math.toRadians(270)))
                    .splineToLinearHeading(new Pose2d(-38,9, Math.toRadians(270)), Math.toRadians(270))
                    //.setTangent(0)
                    .splineToLinearHeading(new Pose2d(-30,9, Math.toRadians(180)), Math.toRadians(0))
            //.setTangent(0)
                    .splineToLinearHeading(new Pose2d(30,9, Math.toRadians(180)), Math.toRadians(0))
                    .splineToLinearHeading(new Pose2d(52.2,28, Math.toRadians(180)), Math.toRadians(0))
            .build());

        // Raise Arm to delivery position
        control.ReadyToLiftSlides();
        sleep(150);
        control.SlidesLow();
        sleep(150);
        control.DeliverPixelToBoardPos();

        // try to find the right April Tag
        control.NavToTag();
        telemetry.addData("Target - ", control.DESIRED_TAG_ID);
        telemetry.addData("rangeError: ",control.rangeError);
        telemetry.addData("yawError: ", control.yawError);
        telemetry.update();
        sleep(2000); //TEMP to debug the values

    // adjust the position to go the correct position for the april tag
    //Actions.runBlocking(
    //       drive.actionBuilder(new Pose2d(50, 36, Math.toRadians(180)))
    //            .splineToLinearHeading(new Pose2d(50+control.rangeError,36+control.yawError, Math.toRadians(180)), Math.toRadians(0))
    //            .build());

        // Drop the pixel on the board
        control.ReleaseRight();
        control.ReleaseLeft();
        sleep(750);

        // Return Arm to Ready position
        control.ResetArm();
        sleep(400);

        // Move the robot to the parking position
        Actions.runBlocking(
                //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                drive.actionBuilder(new Pose2d(52.2, 28, Math.toRadians(180)))
                        .splineToLinearHeading(new Pose2d(50,15, Math.toRadians(270)), Math.toRadians(270))
                        .build());

        telemetry.update();
       // public void TeamArtDelivery()
        {
            Pose2d deliveryPosition;
            Pose2d startPosition;
            //If team art is at left, robot must drive past the spike mark and rotate 45 degrees clockwise and deliver pixel

            // If team art is at center, robot must drive past spike mark and carefully place pixel on spike mark

            // If team art is at right, robot must  drive past spike mark and rotate 45 degrees counterclockwise and deliver the pixel

        }
     //   public void redAudienceTeamArtDelivery()
        {
            if (autoPosition == 1)
            {
                //Pose2d deliverPose = new Pose2d(-44,-29,Math.toRadians(270));
            }
            else if (autoPosition ==2)
            {
                //delivery position = (-36,-20)
            }
            else if (autoPosition == 3)
            {

            }
            else
            {
                //delivery position = (-36,-20)
            }
        }
       // public void redCloseTeamArtDelivery()
        {
            if (autoPosition == 1)
            {
                //delivery position = (-34,-29)
            } else if (autoPosition ==2)
            {

            }
            else if (autoPosition == 3)
            {

            }
            else
            {
                // delivery position = to AutoPosition2
            }
        }
        //public void blueAudienceTeamArtDelivery()
        {
            if (autoPosition == 1)
            {
                //delivery position = (-34,-29)
            } else if (autoPosition ==2)
            {

            }
            else if (autoPosition == 3)
            {

            }
            else
            {
                // delivery position = to AutoPosition2
            }
        }
        //public void blueCloseTeamArtDelivery()
        {
            if (autoPosition == 1)
            {
                //delivery position = (-34,-29)
            } else if (autoPosition ==2)
            {

            }
            else if (autoPosition == 3)
            {

            }
            else
            {
                // delivery position = to AutoPosition2
            }
        }

    }
}