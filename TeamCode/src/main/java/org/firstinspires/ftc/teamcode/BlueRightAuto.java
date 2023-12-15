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
    UniversalControlClass control = new UniversalControlClass(true, false, this);
    private HuskyLens huskyLens;
    private final int READ_PERIOD = 1;
    int leftRightSpikeBound = 150;
    //int autoPosition = 0;
    public double pixelDeliverFirstPos = 14.5;
    MecanumDrive drive;
    Pose2d startPose;
    Pose2d deliverToFloorPose;
    Pose2d deliverToBoardPose;

    public void runOpMode() {
        startPose = new Pose2d(-35.5, 62.5, Math.toRadians(270));

        drive = new MecanumDrive(hardwareMap, startPose);

        control.Init(hardwareMap);
        control.HuskyLensInit();
        control.AutoStartPos();

        telemetry.update();

        //control.WebcamInit(hardwareMap);
        telemetry.update();

        while (!isStarted()) {
            control.DetectTeamArtBlue();
            telemetry.update();
        }
        waitForStart();

        // lock the pixels
        control.GrabPixels();

        // Drives to left, center, or right positions based on team art location.
        BlueRightTeamArtPixelDelivery();

        // drive to the middle floor delivery position
       /* Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .splineToLinearHeading(new Pose2d(-38.5, 12.5, Math.toRadians(270)), Math.toRadians(270))
                        .build());
    */
        // Drop the LEFT pixel (put PURPLE on LEFT, YELLOW on RIGHT) on the line
        control.DropOnLine();
        // put the arm back in a safe to travel position
        control.SafeStow();
        //control.SpecialSleep(10000);

        // drive to the backboard area
        Actions.runBlocking(
                drive.actionBuilder(deliverToFloorPose)
                        .splineToLinearHeading(new Pose2d(-38, 9, Math.toRadians(270)), Math.toRadians(270))
                        //.setTangent(0)
                        .splineToLinearHeading(new Pose2d(-30, 9, Math.toRadians(180)), Math.toRadians(0))
                        //.setTangent(0)
                        .splineToLinearHeading(new Pose2d(30, 9, Math.toRadians(180)), Math.toRadians(0))
                        //  .splineToLinearHeading(new Pose2d(52.2, 28, Math.toRadians(180)), Math.toRadians(0))
                        .build());

        // drive to the correct backboard spot based on the team art
        BlueBoardDelivery();

        //control.ReadyToLiftSlides();
        //sleep(150);
        // Raise Arm to delivery position
        control.SlidesToAuto();
        sleep(150);
        control.DeliverPixelToBoardPos();

        // try to find the right April Tag
       /* control.NavToTag();
        telemetry.addData("Target - ", control.DESIRED_TAG_ID);
        telemetry.addData("rangeError: ", control.rangeError);
        telemetry.addData("yawError: ", control.yawError);
        telemetry.update();
        sleep(2000); //TEMP to debug the values
*/
        // adjust the position to go the correct position for the april tag
        //Actions.runBlocking(
        //       drive.actionBuilder(new Pose2d(50, 36, Math.toRadians(180)))
        //            .splineToLinearHeading(new Pose2d(50+control.rangeError,36+control.yawError, Math.toRadians(180)), Math.toRadians(0))
        //            .build());

        // Drop the pixel on the board
      //  BlueBoardDelivery();
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
                        .splineToLinearHeading(new Pose2d(50, 15, Math.toRadians(270)), Math.toRadians(270))
                        .build());

        telemetry.update();
    }
    // public void TeamArtDelivery()
    //  {
    //   Pose2d deliveryPose;

    //If team art is at left, robot must drive past the spike mark and rotate 45 degrees clockwise and deliver pixel

    // If team art is at center, robot must drive past spike mark and carefully place pixel on spike mark

    // If team art is at right, robot must  drive past spike mark and rotate 45 degrees counterclockwise and deliver the pixel

    //  }
    public void BlueBoardDelivery() {
        // Look for potential errors
        if (control.autoPosition == 1) {
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(new Pose2d(30, 9, Math.toRadians(180)))
                            .splineToLinearHeading(new Pose2d(52.2, 34, Math.toRadians(180)), Math.toRadians(0))
                            .build());
        } else if (control.autoPosition == 2) {

            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(new Pose2d(30, 9, Math.toRadians(180)))
                            .splineToLinearHeading(new Pose2d(52.2, 28, Math.toRadians(180)), Math.toRadians(0))
                            .build());
        } else if (control.autoPosition == 3) {
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(new Pose2d(30, 9, Math.toRadians(180)))
                            .splineToLinearHeading(new Pose2d(52.2, 22, Math.toRadians(180)), Math.toRadians(0))
                            .build());
        } else {
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(new Pose2d(30, 9, Math.toRadians(180)))
                            .splineToLinearHeading(new Pose2d(52.2, 28, Math.toRadians(180)), Math.toRadians(0))
                            .build());
        }
    }

    public void BlueRightTeamArtPixelDelivery() {

        Pose2d aTempPose = new Pose2d(-38.5, 21.5, Math.toRadians(270));

        Actions.runBlocking(
                //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                drive.actionBuilder(startPose)
                        .splineToLinearHeading(aTempPose, Math.toRadians(270))
                        .build());

        if (control.autoPosition == 1) {
            deliverToFloorPose = new Pose2d(-34.5, 32, Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(aTempPose)
                            .splineToLinearHeading(deliverToFloorPose, Math.toRadians(180))
                            .build());
        } else if (control.autoPosition == 2) {
            deliverToFloorPose = new Pose2d(-38.5, 12.5, Math.toRadians(270));

            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(aTempPose)
                            .splineToLinearHeading(deliverToFloorPose, Math.toRadians(270))
                            .build());
        } else if (control.autoPosition == 3) {
            deliverToFloorPose = new Pose2d(-38.5, 20.5, Math.toRadians(315));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  newa Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(aTempPose)
                            .splineToLinearHeading (deliverToFloorPose, Math.toRadians(315))
                            .build());
        } else {
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(-38.5, 12.5, Math.toRadians(270)), Math.toRadians(270))
                            .build());
        }
    }
}
