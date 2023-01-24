# Propulsion
Scripts to enable vehicular propulsion, using physics-based systems

## Source Material

This implementation here is cloned from [vazgriz](https://github.com/vazgriz), specifically, [PID_Controller](https://github.com/vazgriz/PID_Controller). 

It has been cloned to enable the Unity package manager (UPM) to access and update this extension. Other tools in other repositories will benefit from being able to declare UPM dependencies and auto-import the package.


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
