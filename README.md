The Exbus 2 CSRF Telemety convertor acta as a bridge between jeti and crossfire.

Build is relatively simple.

You will need

1.  Teensy 3.2
2.  2 x 2.4k  resistor


The resistors are to enable the two serial ports to operate in single wire mode.

Place resistor across pins 9 and 10.  (TX2 RX2)
Place resitor across pins 0 and 1	(TX1 RX1)

Connect pin 0 on the teensy to the CSRF pin on the crossfire module.
This can be fouund in two locations.

* On the full size module - either on the JR module or the CSRF pin on the module as shown in the Crossfire Manual.
*
* If accessing the pin via the JR module - this is the lowest pin commonly refered to as the ANT or SPORT pin.
*
* An example of this can be found here:
* https://oscarliang.com/ctt/uploads/2016/05/flash-frsky-rx-firmware-Taranis_module_bay.jpg

Connect pin 9 on the teensy to the exbus signal on either the transmitter or the jeti receiver.

* If using a jeti receiver you will use an E1 or E2 port for this function.
* You will need to access the receiver configuration and tell the receiver to output ExBUS.
*
* It is technically also possible to set the internal PPM port on the jeti radio to output exbus.
* I have not testest this - but it should work.

Ensure you have supplied power to the devices and a suitable GND connection is shared.

Enjoy!




