# CySump Schematics

All files were produced using [Fritzing](https://fritzing.org/home/). The following is a list of files and what they are:

- CySump_bb.pdf - A PDF version of the breadboard view.

- CySump_bom.html - The bill of materials in HTML format.

- CySump_pcb.pdf - A PDF version of the PCB view.

- CySump_schem.pdf - A PDF version of the schematic view.

- CySump.fzz - The original Fritzing CAD file.

## NOTES

Firstly, the buzzer and the HCSR04 sensor should *NOT* be mounted to the board. Instead, pin headers should be soldered in there place and then female pin connectors should be used to connect them to the board. This may require making a cable (of sorts) for them and then the buzzer can be mounted somewhere inside whatever housing you use or somewhere external to the board. The sensor needs to be mounted somewhere at the top of the pit. Either by affixing the sensor to a bracket or board that straddles the top of the pit or affixed to the underside of the pit lid, then a cable will need to connect the sensor to the board.

The [Adafruit Power Relay FeatherWing](https://www.adafruit.com/product/3191?gclid=EAIaIQobChMIyfvxycGo4wIVTfDACh0WFg4LEAAYASAAEgL4v_D_BwE) was used because it had a 3V trigger and built-in logic-level and protection circuitry (which means no problems connecting to the ESP8266 and no additional transistors or diodes needed). This board needs to have a pin configured as the 'signal' (trigger) line. I chose pin A which is to the immediate right of the GND pin. This is done by simply soldering the jumper pads next to that pin on the underside of the board.

The power supply used is a 5VDC 1A supply which connects to J2.

NO and COM on the relay should connect to the power line that feeds the pump. This may require cutting and splicing the power cord or modifying an extension cord (the preferred option) and then plugging the pump into the extension. The idea is to use the relay as a power switch. So you want to cut and strip the "hot" wire and connect the 2 ends to the relay.  An even better implementation would be to use a 3-position DPDT Toggle switch rated for at least 120V @ 10A between the power cord and relay. When wired properly, this would allow you to switch between "manual" mode where the pump just runs when needed (or switches over to the float switch that goes with it) and "auto" mode, where CySump then controls the pump in an automated fashion. Personally, I intend to mount an electrical box with a cover plate and toggle switch near the electrical outlet the pump currently plugs into, then cut an extension cord and wire to the switch and CySump controller, with the female end of the extension hanging out so that I can plug the pump into it.

Additionally, I opted to use stacking headers where the pins of the ESP8266 should go (creating a "socket" of sorts) to allow the module to be removable. This is optional, but handy in the event that the module ever becomes damaged or if the EEPROM wears out, or something of that nature, it can be easily replaced.

Lastly, the HCSR04 sensor is NOT waterproof. This presents a problem which I will likely later be resolved with a different sensor. While, I can take some level of precautions by coating most of the module in hot glue or something like that, you cannot cover the cans as they were designed to operate in dry air with nothing in between them. Considering this sensor will be mounted at the top of the pit somewhere or even affixed to the underside of the sump pit lid, sooner or later the moist air will corrode the sensor. As such, the sensor will likely need to be replaced every so often. Since they are cheap ($4 or $5 USD), this isn't an expensive proposition, but should be resolved nonetheless. A future revision will replace this sensor with something more robust and less affected by environmental conditions.
