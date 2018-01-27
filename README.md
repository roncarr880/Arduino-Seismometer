# Arduino-Seismometer

A typical amateur seismometer will have a detector of some sort like a slinky or garden gate style instrument.  Then a circuit to provide filtering and gain.  Then an A/D converter. And then conversion to a serial data stream to a PC running the AMASeis program.   This part of the project describes the filter, gain, A/D conversion, and the serial data parts of the equipment.

The Arduino UNO is used to provide the A/D conversion and serial data stream.  The 10 bit resolution is increased to 14 bits of dynamic range by sampling the signal at 3 different amplitude levels and shifting the bits up where appropriate.
