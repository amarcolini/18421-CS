package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.AbstractComponent
import com.qualcomm.robotcore.hardware.AnalogInput

class DistanceSensor(val input: AnalogInput) : AbstractComponent() {
    fun getDistance(): Double = input.voltage * 204.7244 / input.maxVoltage
}