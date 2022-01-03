
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

```
{
    "type": [string depending on the message to send],
    "content": {
        ... // Depends also on the message to send  
    }
}
```

## Controlling the drone

#### Arming / disarming drone

```
{
    "type": "ARM":
    "content": {
        "armDrone": true / false
    }
}
```
- "true" arms the drone, "false" disarms it

#### Takeoff / landing
```
{
    "type": "TAKEOFF":
    "content": {
        "takeoff": true / false
    }
}

```
- "true" start drone engines, "false" make drone landing
- The drone must be armed before trying to call this command

#### Manual control

This permits to control the drone like a joystick could do.

```
{
    "type": "MANUAL_CONTROL":
    "content": {
        x: // -1.0 = backward,      1.0 = forward
        y: // -1.0 = left,          1.0 = right
        z: // -1.0 = min thrust,    1.0 = max thrust
        r: // -1.0 = clockwise,     1.0 = counter-clockwise
    }
}

```
- All values are ranged between -1.0 and 1.0