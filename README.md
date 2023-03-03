# Propulsion
Scripts to enable vehicular propulsion, using physics-based systems.

## Source Material

The implementation of the PID Controller is sourced from [vazgriz](https://github.com/vazgriz), specifically, [PID_Controller](https://github.com/vazgriz/PID_Controller). 

The implementation of the Backwards PD Controller for Quaternion based rotation control is sourced from [digitalopus](http://digitalopus.ca/site/pd-controllers/).

## Unity Package Manager support /#upm

Add to your project via the Unity Package Manager. 
1. In the Package Manger, select "Add package from Git URL..."
2. Type in 
```
https://github.com/yohash/Propulsion.git#upm
```

The `upm` branch is maintained us a current subtree via:
```
git subtree split --prefix=Assets/Propulsion --branch upm
```
