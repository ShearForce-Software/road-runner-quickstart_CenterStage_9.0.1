package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Disabled
@Autonomous(name="Red Far Multiple Cycles Actions", preselectTeleOp = "1 Manual Control")
public class RedFarMultipleCyclesActions extends LinearOpMode {
    UniversalControlClass control = new UniversalControlClass(true, false,this);
    MecanumDrive drive;
    Pose2d startPose;
    Pose2d deliverToFloorPose;
    Pose2d deliverToBoardPose;
    //Vector2d stackVec = new Vector2d(-56, -12);
    Pose2d stackPose = new Pose2d(-56, -12, Math.toRadians(180));
    public void runOpMode(){
        startPose = new Pose2d(-35.5,-62.5,Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, startPose);
        control.Init(hardwareMap);
        control.HuskyLensInit();
        control.AutoStartPos();

        telemetry.update();

        while(!isStarted()){
            control.DetectTeamArtRed();
            telemetry.update();
        }
        waitForStart();
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        lockPixels(),
                        purpleDelivery()),
                dropOnLine()
                ));
        telemetry.update();
/*
        // lock the pixels
        control.GrabPixels();
        control.ReleaseLeft();

        //Drives to LCR drop on line
        RedLeftTeamArtPixelDelivery();

        // Drop the LEFT pixel (put PURPLE on LEFT, YELLOW on RIGHT) on the line
        control.DropOnLine();
        // put the arm back in a safe to travel position
        control.ResetArmAuto();
       // control.SpecialSleep(8000);

        if(control.autoPosition == 3) {
            Actions.runBlocking(
                    drive.actionBuilder(deliverToFloorPose)
                            .strafeTo(new Vector2d(-36, -12))
                            .splineToLinearHeading(stackPose, Math.toRadians(180))
                            .build());
        }
        else{
            //drive to stack
            Actions.runBlocking(
                    drive.actionBuilder(deliverToFloorPose)
                            .lineToY(-10)
                            .splineToLinearHeading(new Pose2d(-38.5, -12, Math.toRadians(180)), Math.toRadians(0))
                            //.splineToLinearHeading(new Pose2d(-50,-12, Math.toRadians(180)), Math.toRadians(0))
                            .splineToLinearHeading(stackPose, Math.toRadians(180))
                            .build());
        }
        control.AutoPickupRoutine();

        // drive to the backboard area
        drive.updatePoseEstimate();
        Actions.runBlocking(
            drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(180)))
                    .setTangent(0)
                    .splineToLinearHeading(new Pose2d(-30,-9, Math.toRadians(180)), Math.toRadians(0))
                    .splineToLinearHeading(new Pose2d(30,-9, Math.toRadians(180)), Math.toRadians(0))
            .build());

        // drive to the correct backboard spot based on the team art
        RedBoardDelivery();

        // Return Arm to Ready position
        control.SlidesToAuto();
        sleep(150);
        control.DeliverPixelToBoardPos();
        control.StopNearBoardAuto(true);
        sleep(200);

        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(180)))
                        .lineToX(47)
                        .build());

        control.ResetArmAuto();
        sleep(200);

        //drive back to stack
        drive.updatePoseEstimate();
        Actions.runBlocking(
                //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(180)))
                        .setReversed(true)
                        .setTangent(180)
                        .splineToLinearHeading(new Pose2d(30,-9, Math.toRadians(180)), Math.toRadians(180))
                        //.setTangent(180)
                        //.splineToLinearHeading(new Pose2d(-50,-12, Math.toRadians(180)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(-50,-12, Math.toRadians(180)), Math.toRadians(180))
                        .build());
        control.AutoPickupRoutine();

        //drive back to board
        drive.updatePoseEstimate();
        Actions.runBlocking(
                //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(180)))
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(-30,-9, Math.toRadians(180)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(30,-9, Math.toRadians(180)), Math.toRadians(0))
                        .build());
        RedBoardDelivery();
        // Return Arm to Ready position
        control.SlidesToAuto();
        sleep(150);
        control.DeliverPixelToBoardPos();
        control.StopNearBoardAuto(true);
        sleep(200);

        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(180)))
                        .lineToX(47)
                        .build());

        control.ResetArmAuto();
        sleep(200);

        // Move the robot to the parking position
        Actions.runBlocking(
                //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                drive.actionBuilder(new Pose2d(50, -28, Math.toRadians(180)))
                        .splineToLinearHeading(new Pose2d(48,-15, Math.toRadians(90)), Math.toRadians(90))
                        .build());
        control.ServoStop();
        sleep(100);
        telemetry.update();*/
    }
    public void RedBoardDelivery() {
        // Look for potential errors
        //***POSITION 1***
        if (control.autoPosition == 1) {
            deliverToBoardPose = new Pose2d(50,-28,Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(new Pose2d(30,-9, Math.toRadians(180)))
                            .setTangent(0)
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0))
                            .build());
        }
        //***POSITION 3***
        else if (control.autoPosition == 3) {
            deliverToBoardPose = new Pose2d(50,-38,Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(new Pose2d(30, -9, Math.toRadians(180)))
                            .setTangent(0)
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0))
                            .build());
        }
        //***POSITION 2***
        else {
            deliverToBoardPose = new Pose2d(50,-33,Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(new Pose2d(30, -9, Math.toRadians(180)))
                            .setTangent(0)
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0))
                            .build());
        }
    }
    public void RedLeftTeamArtPixelDelivery() {
        Pose2d aTempPose = new Pose2d(-38.5, -33, Math.toRadians(90));

        Actions.runBlocking(
                //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                drive.actionBuilder(startPose)
                        .splineToLinearHeading(aTempPose, Math.toRadians(90))
                        .build());
        //***POSITION 1***
        if (control.autoPosition == 1) {
            deliverToFloorPose = new Pose2d(-38.5, -20.5, Math.toRadians(45));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  newa Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(aTempPose)
                            .splineToLinearHeading (deliverToFloorPose, Math.toRadians(45))
                            .build());

        }
        //***POSITION 3***
        else if (control.autoPosition == 3) {
            deliverToFloorPose = new Pose2d(-34.5, -32, Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(aTempPose)
                            .splineToLinearHeading(new Pose2d(-27, -33, Math.toRadians(180)), Math.toRadians(180))
                            .splineToLinearHeading(deliverToFloorPose, Math.toRadians(180))
                            .build());
        }
        //***POSITION 2***
        else {
            deliverToFloorPose = new Pose2d(-38.5, -12.5, Math.toRadians(90));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(aTempPose)
                            .splineToLinearHeading(deliverToFloorPose, Math.toRadians(90))
                            .build());
        }
    }
    public Action purpleDelivery(){return new PurpleDelivery();}
    public class PurpleDelivery implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
                drive.updatePoseEstimate();
                if (control.autoPosition == 1) {
                    deliverToFloorPose = new Pose2d(-39.5, -20.5, Math.toRadians(45));
                    /*TrajectoryActionBuilder FirstCone = drive.actionBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                            .splineToLinearHeading (deliverToFloorPose, Math.toRadians(45))
                            .build());*/
                    Actions.runBlocking(
                            //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                            drive.actionBuilder(startPose)
                                    .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                                    .splineToLinearHeading (deliverToFloorPose, Math.toRadians(45))
                                    .build());
                }
                //***POSITION 3***
                else if (control.autoPosition == 3) {
                    deliverToFloorPose = new Pose2d(-34.5, -32, Math.toRadians(180));
                    Actions.runBlocking(
                            //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                            drive.actionBuilder(startPose)
                                    .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                                    .splineToLinearHeading(new Pose2d(-27, -33, Math.toRadians(180)), Math.toRadians(180))
                                    .splineToLinearHeading(deliverToFloorPose, Math.toRadians(180))
                                    .build());
                }
                //***POSITION 2***
                else {
                    deliverToFloorPose = new Pose2d(-38.5, -12.5, Math.toRadians(90));
                    Actions.runBlocking(
                            //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                            drive.actionBuilder(startPose)
                                    .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                                    .splineToLinearHeading(deliverToFloorPose, Math.toRadians(90))
                                    .build());
                }
            }
            packet.put("deliver purple pixel", 0);
            boolean returnValue = false;

            drive.updatePoseEstimate();
            if ((drive.pose.position.x == deliverToFloorPose.position.x) && (drive.pose.position.y == deliverToFloorPose.position.y))
            {
                returnValue = false;
            }
            return returnValue;
        }
    }
    public Action lockPixels(){return new LockPixels();}
    public class LockPixels implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.GrabPixels();
                control.ReleaseLeft();
                initialized = true;
            }
            packet.put("lock purple pixel", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }
    public Action dropOnLine(){return new DropOnLine();}
    public class DropOnLine implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                control.DropOnLine();
                initialized = true;
            }
            packet.put("drop purple pixel on line", 0);
            return false;  // returning true means not done, and will be called again.  False means action is completely done
        }
    }

}
