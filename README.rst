






To Do
-----

* Store settings for IMU to write on startup.

  * There is no settings storage on the IMU (according to the datasheet), so any changes need to be written after
startup

* Store offset matrices and values on Adruino

  * There are offset registers on the IMU, but they don't seem to work

* Add interrupt mode in addition to state machine mode

  * Some methods, such as magnetometer reading, require the state machine, due to waiting for data. However, if
interrupts are used, callbacks can be set to run on those interrupts and get the magnetometer or accelerometer data
when it's ready.

  * You could run the read functions in a main loop, however, there's no guarantee that the data is updated, or that
it's handled right with a callback when it's new data instead of using old data. The interrupt or state machine methods
should handle these cases correctly while also being as fast as possible.