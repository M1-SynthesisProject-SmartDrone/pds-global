
```
     ___      .______    __                   ___      .______   .______          .______     ______ 
    /   \     |   _  \  |  |                 /   \     |   _  \  |   _  \         |   _  \   /      |
   /  ^  \    |  |_)  | |  |                /  ^  \    |  |_)  | |  |_)  |  ______|  |_)  | |  ,----'
  /  /_\  \   |   ___/  |  |               /  /_\  \   |   ___/  |   ___/  |______|   ___/  |  |     
 /  _____  \  |  |      |  |              /  _____  \  |  |      |  |             |  |      |  `----.
/__/     \__\ | _|      |__|             /__/     \__\ | _|      | _|             | _|       \______|
                                                                                                     


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

If a reponse is returned, it will have this shape :

```json
{
    "type": "ANSWER",
    "content": {
        "name": "[The name describing the message response]",
        "validated": true,
        "message": "[optional string containing some informations]"
    }
}
```
> A response is not garanteed to come directly after the call (this is up to the drone)

## Controlling the drone

#### Arming & launching the drone

In order to start to use the drone, only one command must be sent :

```json
{
    "type": "START_DRONE",
    "content": {
        "startDrone": true // false
    }
}
```

- If startDrone is set to "true", the drone will try to arm and be ready to fly
- "false" does nothing
- The user have 10 seconds to takeoff the drone (putting motor thrusts at a value >= 62.5%)
- In order to stop it, one simply have to land the drone, the disarm is automatic

#### Manual control

This permits to control the drone like a joystick could do.

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

#### Record flights

This single action permits to start and stop flight recording

```json
{
    "type": "RECORD",
    "content": {
        "record": true // false
    }
}
```
> This message will trigger a response

- "true" will start recording or throw an error if drone is unarmed
- "false" will stop the recording (only if one was started)

# Ground station

The ground station could (and will !) send data periodically about the drone data.

## Drone data

The latitude / longitude are expressed in integer as "degE7" format : this seems to be only a way to express a fixed-point floating variable. For example :

```java
int degE7 = 123456789;
float converted = 12.3456789;
```

So, we just need to work a bit on the received number in order to use it.

```json
{
    "type": "DRONE_DATA",
    "content": {
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

## Drone state

Periodically, the drone will send a message indicating if it is armed or not. This message is re-sent to the application.
This message could have more content than motors state after time.

```json
{
    "type": "DRONE_STATE",
    "content": {
        "armed": true // false
    }
}
```