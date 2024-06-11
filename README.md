### Engine interface

This program connects a classic engine ot signalK.

## Tachometer

The tachometer data is read from the alternator. It is a more or less square signal of 15 v. 

This signal frequency is the speed of the alternator * number of poles. The speed  of the alternator
is determined by the speed of the motor and the relation of the wheels used to drive it.

The data is sent in Hz to signalK to self.propulsion.1.revolutions-

Also if frequency is 0 the propulsion.1.state is stopped, if it is not zero is sterted