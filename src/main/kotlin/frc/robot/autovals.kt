package frc.robot

enum class autovals (xpos: Double, ypos:Double) {
    /**
     * ALL of the positions needed to place depending on where one is one field
     * high meaning closest to charge station, low meaning farthest
     */
    zonehigh(2.0, 1.05),
    zonemid(2.0, 2.75),
    zonelow(2.0, 2.0),
    pickhigh(6.4, 4.6),
    pickmid(6.4, 2.15),
    picklow(6.4, .95),
}