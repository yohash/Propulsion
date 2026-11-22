# Third Party Notices

This package contains code adapted from the third-party projects below. Each is used under the license reproduced here.

## PID_Controller

- Source: https://github.com/vazgriz/PID_Controller (see also https://vazgriz.com/621/pid-controllers/)
- Author: vazgriz
- License: MIT
- Used in: `Runtime/Propulsion/PidController.cs`

The upstream license does not name a copyright holder; it is reproduced verbatim below.

```
The MIT License (MIT)

Copyright (c) 2021 

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## UsefulUtils (digitalopus)

- Source: https://github.com/Phong13/UsefulUtils (see also http://digitalopus.ca/site/pd-controllers/)
- Author: Ian Deane
- License: MIT
- Used in: `Runtime/Propulsion/StablePdController.cs` and `Runtime/Propulsion/PropulsionMath.cs` (the quaternion and inertia-tensor handling)

```
MIT License

Copyright (c) 2023 Ian Deane

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Stable Proportional-Derivative Controllers

`StablePdController` is based on the method described in:

> J. Tan, K. Liu, G. Turk. *Stable Proportional-Derivative Controllers*. http://www.jie-tan.net/project/spd.pdf

No code or text from the paper is included in this package. A reference copy is kept in the source repository, under `Documentation/`, outside the package; its copyright belongs to its authors and publisher.
