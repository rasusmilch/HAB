
Repository Contents
-------------------
The following files contain different code that can be loaded either onto an Arduino board to interact with the Geiger counter, or to the Geiger Counter itself. 
You can compile the code by going to a command line and typing "make" for the files. 

* **/Arduino_Ethernet_Board** - directory contains firmware for an Arduino ethernet board connected to the Cosm.com API. The Geiger counter sends data to to the ethernet board, CPM is calcluated, then every minute CPM is loaded to the cosm servers. The live SparkFun Geiger Counter feed can be found [here](https://cosm.com/feeds/22279).
* **/Geiger_Counter_Board** - directory contains the firmware for the SparkFun Geiger Counter board. This firmware takes the pulses from the Geiger tube and outputs a random bit (0 or 1), every time an event occurs. (V13)
* **/Default_Firmware** - directory contains the production hex file that comes loaded onto every Gieger Counter (V12 and V13)