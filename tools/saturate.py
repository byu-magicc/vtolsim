"""
Saturation function:
    the output is the input saturated between low_limit and up_limit
"""

def saturate(input: float, 
             low_limit: float, 
             up_limit: float)->float:
    if input <= low_limit:
        output = low_limit
    elif input >= up_limit:
        output = up_limit
    else:
        output = input
    return output
