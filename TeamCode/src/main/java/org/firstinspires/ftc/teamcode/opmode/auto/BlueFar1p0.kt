package org.firstinspires.ftc.teamcode.opmode.auto

import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.CSRobot
import org.firstinspires.ftc.teamcode.tile
import org.firstinspires.ftc.teamcode.vision.PropPipeline

@Autonomous
@JoosConfig
class BlueFar1p0 : CommandOpMode() {
    private val robot by robot<CSRobot>()
    private val pipeline = PropPipeline(false)

    companion object {
        var startPose = Pose2d(16.0, 3 * tile - 9.0, (-90).deg)
        var rightPlopPose = Pose2d(10.0, 35.0, (180).deg)
        var centerPlopPose = Pose2d(16.0, 37.0, (-90).deg)
        var leftPlopPose = Pose2d(26.0, 46.0, (-90).deg)
    }

    override fun preInit() {
        robot.outtake.init()
        robot.pixelPlopper.prime()
        robot.openFrontCameraAsync {
            robot.frontCamera.setPipeline(pipeline)
            schedule(true) {
                telem.addData("prop location", pipeline.lastKnownLocation)
            }
        }
    }

    override fun preStart() {
        cancelAll()
        robot.outtake.ready()
        robot.drive.poseEstimate = startPose

        val plopPose = when (pipeline.lastKnownLocation) {
            PropPipeline.PropLocation.Left -> leftPlopPose
            PropPipeline.PropLocation.Center -> centerPlopPose
            PropPipeline.PropLocation.Right -> rightPlopPose
        }
        //Just placing purple
        val purplePlopCommand = robot.drive.pathCommandBuilder(startPose)
            .lineToSplineHeading(plopPose)
            .then(robot.pixelPlopper.plop())
            .back(3.0)
            .build()
        purplePlopCommand.thenStopOpMode().schedule()
    }
}