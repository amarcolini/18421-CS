package org.firstinspires.ftc.teamcode.opmode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.command.SequentialCommand
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.CSRobot
import org.firstinspires.ftc.teamcode.tile
import org.firstinspires.ftc.teamcode.vision.PropPipeline
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation

@Autonomous
@Disabled
@JoosConfig
class RedFarAuto : CommandOpMode() {
    private val robot by robot<CSRobot>()
    private val pipeline = PropPipeline(true)

    companion object {
        var startPose = Pose2d(16.0, -3 * tile + 9.0, (90).deg)
        var rightPlopPose = Pose2d(30.0, -50.0, (90).deg)
        var rightPlacePose = Pose2d(51.0, -48.0, 0.deg)
        var centerPlopPose = Pose2d(16.0, -36.0, (90).deg)
        var centerPlacePose = Pose2d(51.0, -42.0, 0.deg)
        var leftPlopPose = Pose2d(10.5, -36.0, (180).deg)
        var leftPlacePose = Pose2d(51.0, -36.0, 0.deg)
    }

    override fun preInit() {
        robot.outtake.ready()
        robot.pixelPlopper.prime()
        robot.webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                robot.webcam.startStreaming(
                    640,
                    480,
                    OpenCvCameraRotation.UPRIGHT
                )
                robot.webcam.setPipeline(pipeline)
                FtcDashboard.getInstance().startCameraStream(robot.webcam, 0.0)

                schedule(true) {
                    telem.addData("prop location", pipeline.lastKnownLocation)
                }
            }

            override fun onError(errorCode: Int) {
                telem.addLine("Camera failed to open!!").setRetained(true)
                telem.update()
                requestOpModeStop()
            }

        })
    }

    override fun preStart() {
        cancelAll()
        robot.drive.poseEstimate = startPose

        val (plopPose, placePose) = when (pipeline.lastKnownLocation) {
            PropPipeline.PropLocation.Left -> leftPlopPose to leftPlacePose
            PropPipeline.PropLocation.Center -> centerPlopPose to centerPlacePose
            PropPipeline.PropLocation.Right -> rightPlopPose to rightPlacePose
        }
        val purplePlopTrajectory =
            robot.drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(plopPose)
                .build()
        val yellowPlaceTrajectory =
            robot.drive.trajectoryBuilder(purplePlopTrajectory.end())
                .lineToSplineHeading(placePose)
                .build()

        val purplePlopCommand = robot.drive.followTrajectory(purplePlopTrajectory)
            .then(robot.pixelPlopper.plop())

        val yellowPlaceCommand = robot.drive.followTrajectory(yellowPlaceTrajectory)
            .then(robot.outtake::extend)
            .wait(2.0)
            .then(robot.outtake::releaseRight)
            .wait(2.0)
            .then(robot.outtake::reset)
            .wait(2.0)

        SequentialCommand(
            true,
            purplePlopCommand,
//            yellowPlaceCommand
        ).thenStopOpMode().schedule()
    }
}