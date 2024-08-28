#saves all of the trim values for the quadplane anaconda airframe

from message_types.quad.msg_delta import MsgDelta


#saves each of the trims

elevatorTrim = -0.124778
aileronTrim = 0.001836
rudderTrim = -0.000303
forwardThrottleTrim = 0.676752

#--------------------------------------------------------------------------
#------------------------------TODO----------------------------------------
#we need to figure out the trim throttles for the vertical throttles
verticalThrottle_1_Trim = 0.0
verticalThrottle_2_Trim = 0.0
verticalThrottle_3_Trim = 0.0
verticalThrottle_4_Trim = 0.0

#saves the trim delta
trimDelta = MsgDelta(elevator=elevatorTrim,
                     aileron=aileronTrim,
                     rudder=rudderTrim,
                     forwardThrottle=forwardThrottleTrim,
                     verticalThrottle_1=verticalThrottle_1_Trim,
                     verticalThrottle_2=verticalThrottle_2_Trim,
                     verticalThrottle_3=verticalThrottle_3_Trim,
                     verticalThrottle_4=verticalThrottle_4_Trim)
