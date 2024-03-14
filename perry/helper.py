def getInterpAng(distance):
    """returns angle for shots farther from first stage truss"""
    # meters : degrees above horizontal
    # random numbers
    distMap = {
        2: 45,
        3: 37,
        4: 22,
        4.5: 40,
        5: 13
    }

    if distance in distMap:
        return distMap[distance]

    # get closest key
    res_key, res_val = min(distMap.items(), key=lambda x: abs(distance - x[0]))
    
    # check if key on the end or beginning of dict
    keys = list(distMap.keys())
    if res_key == max(keys):
        dydx = (distMap[keys[-2]] - distMap[keys[-1]]) / (keys[-2] - keys[-1])
        return dydx * (distance - keys[-1]) + distMap[keys[-1]]
    

    elif res_key == min(keys):
        dydx = (distMap[keys[1]] - distMap[keys[0]]) / (keys[1] - keys[0])
        return dydx * (distance - keys[0]) + distMap[keys[0]]

    # interpolate
    if distance > res_key:
        # interpolate between res_val and next val
        next_key = keys[keys.index(res_key) + 1]
        dydx = (distMap[next_key] - res_val) / (next_key - res_key)
        return dydx * (distance - res_key) + res_val

    elif distance < res_key:
        # interpolate between res_val and prev val
        prev_key = keys[keys.index(res_key) - 1]
        dydx = (res_val - distMap[prev_key]) / (res_key - prev_key)
        return dydx * (distance - prev_key) + distMap[prev_key]


import rev

def tempProt(motorController: rev.CANSparkMax):
    """Check if motor temp is too high before doing stuff, nonzero eror code means motor too hot"""
    temp = motorController.getMotorTemperature()
    if temp > 100:
        motorController.set(0)
        return 1
    
    return 0