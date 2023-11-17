package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.dashboard.ConfigHandler
import com.amarcolini.joos.extensions.telem
import com.amarcolini.joos.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.reflect.full.createType
import kotlin.reflect.jvm.jvmErasure

@TeleOp
class ConfigHandlerLogs : LinearOpMode() {
    private val taco: Pose2d = Pose2d()

    override fun runOpMode() {
        telem.addLine("Pose2d annotations: ${Pose2d::class.annotations}")
        telem.addLine("Pose2d type annotations: ${Pose2d::class.createType().annotations}")
        telem.addLine("Pose2d variable annotations: ${::taco.returnType.jvmErasure.annotations}")
        ConfigHandler.getLogs().forEach {
            telem.addLine(it).setRetained(true)
        }
        telem.update()

        waitForStart()

        while (opModeIsActive()) {
            sleep(500)
        }
    }
}