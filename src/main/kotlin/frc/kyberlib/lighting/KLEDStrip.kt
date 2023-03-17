package frc.kyberlib.lighting

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import frc.kyberlib.command.Game
import java.awt.Color

class KLEDStrip(port: Int, private val length: Int) {

    companion object {
        private var singleton: KLEDStrip? = null
    }


    private val addressableLED = AddressableLED(port)
    private val regions = arrayListOf<KLEDRegion>()

    private val startTime = Game.time

    init {
        require(singleton == null) { "Only one LED strip may be used at once" }
        addressableLED.setLength(length)
        addressableLED.start()
    }

    operator fun plusAssign(other: KLEDRegion) {
        require(other.end > other.start, { "Region end must be greater than start" })
        require(other.start >= 0, { "Region start must be greater than or equal to 0" })
        require(other.end <= length, { "Region end must be less than or equal to the length of the strip" })

        regions.add(other)
    }

    val buffer = AddressableLEDBuffer(length)
    val mutableBuffer = Array(length) { Color.black }
    fun update() {
        val dt = Game.time - startTime
        KLEDRegion.optimizedComposite(mutableBuffer, dt, regions)
        mutableBuffer.forEachIndexed { index, color ->
            mutableBuffer[index] = mutableBuffer[index].gammaCorrect()
        }

        for (i in mutableBuffer.indices) {
            buffer.setRGB(
                i,
                mutableBuffer[i].red,
                mutableBuffer[i].green,
                mutableBuffer[i].blue
            )
        }

        addressableLED.setData(buffer)
    }
}
