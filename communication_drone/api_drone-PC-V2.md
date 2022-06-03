
```
           _____ _____            _____  _____        _____   _____  __      _____  
     /\   |  __ \_   _|     /\   |  __ \|  __ \      |  __ \ / ____| \ \    / /__ \ 
    /  \  | |__) || |      /  \  | |__) | |__) |_____| |__) | |       \ \  / /   ) |
   / /\ \ |  ___/ | |     / /\ \ |  ___/|  ___/______|  ___/| |        \ \/ /   / / 
  / ____ \| |    _| |_   / ____ \| |    | |          | |    | |____     \  /   / /_ 
 /_/    \_\_|   |_____| /_/    \_\_|    |_|          |_|     \_____|     \/   |____|
                                                                                    

```
All json-formatted objects that the app can send to the ground station.

## Base structure

All objects will have the following base format : 

```json
{
    "type": "[string depending on the message to send]",
    "content": {
        "...": "..." // Depends also on the message to send  
    }
}
```

The responses types will have the type "RESP_[RequestType]"

## Connection / Acknoledgement

If the app wants to know if the server is here, it can send this message :

### Request
```json
{
    "type": "ACK",
    "content": {
    }
}
```

### Response
```json
{
    "type": "RESP_ACK",
    "content": {
        "validated": true,
        "message": ""
    }
}
```
> The response should come rapidly.

## Controlling the drone

### Arming & launching the drone

In order to start to use the drone, only one command must be sent :

#### Request
```json
{
    "type": "START_DRONE",
    "content": {
        "startDrone": true // false
    }
}
```


#### Response
```json
{
    "type": "RESP_START_DRONE",
    "content": {
        "validated": true, // false
        "message": ""
    }
}
```

- If startDrone is set to "true", the drone will try to arm and be ready to fly
- "false" does nothing
- The user have 10 seconds to takeoff the drone (putting motor thrusts at a value >= 62.5%)
- In order to stop it, one simply have to land the drone, the disarm is automatic

### Manual control

This permits to control the drone like a joystick could do.

#### Request
```json
{
    "type": "MANUAL_CONTROL",
    "content": {
        "x":  0.0,  // -1.0 = backward,      1.0 = forward
        "y":  0.0,  // -1.0 = left,          1.0 = right
        "z":  0.0,  // -1.0 = min thrust,    1.0 = max thrust
        "r":  0.0   // -1.0 = clockwise,     1.0 = counter-clockwise
    }
}
```
- All values are ranged between -1.0 and 1.0

#### Response

No response is returned for this request

### Record flights

This single action permits to start and stop flight recording

#### Request

```json
{
    "type": "RECORD",
    "content": {
        "record": true // false
    }
}
```
- "true" will start recording or throw an error if drone is unarmed
- "false" will stop the recording (only if one was started)

#### Response

```json
{
    "type": "RESP_RECORD",
    "content": {
        "validated": true, // false
        "message": ""
    }
}
```

## Receiving drone data

The user must send requests periodically in order to update accordignly the drone state and infos

### Receiving drone informations

#### Request
```json
{
    "type": "DRONE_INFOS",
    "content": {}
}
```

#### Response

The latitude / longitude are expressed in integer as "degE7" format : this seems to be only a way to express a fixed-point floating variable. For example :

```java
int degE7 = 123456789;
float converted = 12.3456789;
```

So, we just need to work a bit on the received number in order to use it.

```json
{
    "type": "RESP_DRONE_INFOS",
    "content": {
        "armed": true,
        "recording": false,
        "batteryRemaining": 50, // (int) [0, 100]
        "lat": 12345678, // (int, degE7)
        "lon": 12345678, // (int, degE7)
        "alt": 1223, // altitude above sea-level (mm)
        "relativeAlt": 365, // altitude above ground (mm)
        "vx": 34, // Ground speed (cm/s)
        "vy": 12, // Ground speed (cm/s)
        "vz": 3, // Ground speed (cm/s)
        "yawRotation": 45 // Drone rotation (cdeg)
    }
}
```

# Handling paths

## Get all paths

#### Request 

```json
{
    "type": "PATH_LIST",
    "content": {}
}
```

#### Response
```json
{
    "type": "RESP_PATH_GET",
    "content": {
        "paths": [
            {
                "name": "path n°1",
                "id": 1,
                "date": "2022-03-28",
            },
            ...
        ]
    }
}
```

## Get a particular path

#### Request

```json
{
    "type": "PATH_ONE",
    "content": {
        "pathId": 1
    }
}
```

#### Response

```json
{
    "type": "RESP_PATH_ONE",
    "content": {
        "id": 1,
        "name": "path n°1",
        "date": "2022...",
        "nbPoints": 210,
        "nbCheckpoints": 50,
        "departure": {
            "lat": "12.3456789",
            "lon": "12.3456789",
            "alt": "202"
        }
    }
}
```
> If the path does not exist, an empty content will be sent

## Launch a path

#### Request

```json
{
    "type": "PATH_LAUNCH",
    "content": {
        "pathId": 1
    }
}
```

#### Response

```json
{
    "type": "RESP_PATH_LAUNCH",
    "content": {
        "validated": true
    }
}
```

# In autopilot mode

## Get informations (same as DRONE_INFOS)

#### Request

```json
{
    "type": "AUTOPILOT_INFOS",
    "content": {}
}
```

#### Response

Response is the same as "RESP_DRONE_INFOS", but with more infos

```json
{
    "type": "RESP_AUTOPILOT_INFOS",
    "content": {
        "armed": true,
        "recording": false,
        "batteryRemaining": 50, // (int) [0, 100]
        "lat": 12345678, // (int, degE7)
        "lon": 12345678, // (int, degE7)
        "alt": 1223, // altitude above sea-level (mm)
        "relativeAlt": 365, // altitude above ground (mm)
        "vx": 34, // Ground speed (cm/s)
        "vy": 12, // Ground speed (cm/s)
        "vz": 3, // Ground speed (cm/s)
        "yawRotation": 45, // Drone rotation (cdeg)
        // NEW
        "running": true, // The autopilot part is running (when false, the trip is ended)
        "errorMode": false,
        "manualControl": false,
    }
}
```

- If "errorMode" and "waitForControl" are both true, this means that the drone 
is stopped and waits for the user to choose to continue or to take control

- If user chose to resume, the "errorMode" will be re-set to false

- If the user chose to take control, the "errorMode" will still be let to true 
but the "waitForControl" will be set to false


## Take control while in error mode

#### Request

```json
{
    "type": "REGAIN_CONTROL",
    "content": {}
}
```

#### Response

```json
{
    "type": "RESP_REGAIN_CONTROL",
    "content": {
        "validated": true,
        "message": ""
    }
}
```

After that, you can manual control the drone like before, 
and register is automatically done. Be careful to still use "AUTOPILOT_INFOS" request

## Resume the autopilot

#### Request

```json
{
    "type": "RESUME_AUTOPILOT",
    "content": {}
}
```

#### Response

```json
{
    "type": "RESP_RESUME_AUTOPILOT",
    "content": {
        "validated": true,
        "message": ""
    }
}
```

