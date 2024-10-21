
#imports the delta message class
from message_types.msg_delta import MsgDelta


#stores the trim values for the four controls of the aerosonde

throttleTrim = 0.676752
aileronTrim = 0.001836
elevatorTrim = -0.124778
rudderTrim = -0.000303
vertical1Trim = 0.0
vertical2Trim = 0.0
vertical3Trim = 0.0
vertical4Trim = 0.0


#instantiates the Message delta class
trimDelta = MsgDelta(elevator=elevatorTrim, 
                     aileron=aileronTrim, 
                     rudder=rudderTrim, 
                     forwardThrottle=throttleTrim,
                     verticalThrottle_1=vertical1Trim,
                     verticalThrottle_2=vertical2Trim,
                     verticalThrottle_3=vertical3Trim,
                     verticalThrottle_4=vertical4Trim)



