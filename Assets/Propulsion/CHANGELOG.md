# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.3.2] - 2023-03-07

### Changed

- Modified the Pid Example scene to compare the PID controllers to the backwards PD controller

## [0.3.1] - 2023-03-06

### Changed

- Inputs for Backwards PD rotation controller reduced to physics values to remove dependence on Rigidbody. This allows the algorithm to be used with an ArticulationBody, or a custom solution

## [0.3.0] - 2023-03-03

### Added

- Backwards PD Controller for quaternion based rotation control from [digitalopus.ca/site/pd-controllers/](http://digitalopus.ca/site/pd-controllers/)