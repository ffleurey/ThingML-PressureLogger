# Testing DPS310 Pressure sensors

For this test we are using two DPS310 pressure sensors. Both sensors are connected to the I2C bus of an arduino.

We are intereted in the variations in altitude between the two sensors.

The pressure is logged at around 10Hz and the pressure difference between the two sensors is calculated and low pass filtered. 


* **Case A** corresponds to a variation in pressure in the room (opening the window). Both sensrors measure the same variation in pressure so the difference (at the bottom) remains 0.
 * **Case B** coresponds to moving sensor B about 1 meter up and back. The decrease in pressure is clearly visible. Sensor A is not affected and the difference reflect accuraly the change in relative altitude between the 2 sensors.
 * **Case C** coresponds to warming up sensor B by placing a cup of coffee next to it. Bothe sensors are kept stationory. Exact variation in temperature has not been measured. The graph shows that the pressure reading is affected and the presure difference graph reflects it.

Some form of temperature compensation seem necessary in order to ensure that pressure differences are related to changes in relative altitude and not temperature changes.