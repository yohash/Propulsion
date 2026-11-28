# Propulsion
Scripts to enable vehicular propulsion, using physics-based systems.

This repository holds one or more Unity packages, each in its own folder under [`Packages/`](Packages), and a Unity project for developing them. Each package has its own README, CHANGELOG, UPM branch and release tags.

| Package | Folder | Install (UPM branch) | Release tags |
|---|---|---|---|
| `com.yohash.propulsion`: MonoBehaviour controllers, vehicle and samples | [`Packages/com.yohash.propulsion`](Packages/com.yohash.propulsion) ([README](Packages/com.yohash.propulsion/README.md), [CHANGELOG](Packages/com.yohash.propulsion/CHANGELOG.md)) | `#upm` | `vX.Y.Z` |
| `com.yohash.propulsion.core`: the Burst-compatible math core, over Unity.Mathematics | [`Packages/com.yohash.propulsion.core`](Packages/com.yohash.propulsion.core) ([README](Packages/com.yohash.propulsion.core/README.md), [CHANGELOG](Packages/com.yohash.propulsion.core/CHANGELOG.md)) | `#upm-core` | `core-vX.Y.Z` |
| `com.yohash.propulsion.entities`: ECS components and systems, with a backend per physics package (ECSPhysics2D) | [`Packages/com.yohash.propulsion.entities`](Packages/com.yohash.propulsion.entities) ([README](Packages/com.yohash.propulsion.entities/README.md), [CHANGELOG](Packages/com.yohash.propulsion.entities/CHANGELOG.md)) | `#upm-entities` | `entities-vX.Y.Z` |

## Source Material

The implementation of the PID Controller is sourced from [vazgriz](https://github.com/vazgriz), specifically, [PID_Controller](https://github.com/vazgriz/PID_Controller). 

The Stable PD Controller is based on [Stable Proportional-Derivative Controllers](http://www.jie-tan.net/project/spd.pdf) (Tan, Liu & Turk) and the "backwards" PD controller from [digitalopus](http://digitalopus.ca/site/pd-controllers/) ([UsefulUtils](https://github.com/Phong13/UsefulUtils)). Its gains are solved exactly for Unity's physics integrator, so the response has the requested frequency and damping at any timestep, and it is unconditionally stable.

## Controllers

| Controller | Use it for |
|---|---|
| `StablePdController` | Matching a target position / velocity, or rotation / angular velocity, in 1 or 3 axes. Physically parameterized (frequency in Hz, damping ratio), scaled by mass or inertia, unconditionally stable. Known forces such as gravity can be cancelled exactly through its feedforward term. |
| `PidController` | A bounded, normalized throttle with an integral term, for actuators with limited strength or unknown persistent disturbances. Hand tuned. |
| `PropulsionMath` | Shared helpers: angle wrapping, quaternion to rotation vector, angular velocity from successive rotations, angular acceleration to torque. |

## Reference material

Papers and other background material live in [`Documentation/`](Documentation). They are kept out of the Unity package itself.

## Installation (Unity Package Manager)

Requires Unity 6 (6000.3) or newer.

1. In the Package Manager, select **+ > Add package from git URL...**
2. Enter the package's UPM branch, or a release tag to pin a version:
```
https://github.com/yohash/Propulsion.git#upm
https://github.com/yohash/Propulsion.git#v0.6.0
https://github.com/yohash/Propulsion.git#upm-core
https://github.com/yohash/Propulsion.git#upm-entities
```

When a package depends on another package from this repository, install the dependency first. For example, `com.yohash.propulsion.entities` needs `com.yohash.propulsion.core`, and a physics package for its backend, installed first. Unity resolves `package.json` dependencies only from package registries, not from git URLs. Each package's README lists what to install.

## Repository layout and development

```
Packages/
  com.yohash.propulsion/       an embedded package: package.json, Runtime/, Tests/, Samples~/, docs
  com.yohash.propulsion.core/  an embedded package: package.json, Runtime/, Tests/, docs
  com.yohash.propulsion.entities/
                               an embedded package: Runtime/, one folder per physics backend (Physics2D/), Tests/, Samples~/, docs
  manifest.json                the development project's manifest (lists packages under testables)
Assets/                        the development project's assets (empty; Unity requires the folder)
Documentation/                 reference material, kept out of every package
.github/workflows/             publishing (see Releasing)
```

Opening the repository in Unity loads each folder under `Packages/` as an **embedded package**. Unity validates its `package.json` dependencies and assembly references exactly as it will for consumers. Each package is listed under `testables` in `Packages/manifest.json`, so its tests appear in **Window > General > Test Runner**. The project also installs Entities and ECSPhysics2D (pinned to a release tag), so the entities package's ECSPhysics2D backend and its engine tests compile here.

**Working on samples.** Unity hides folders ending in `~`, so a package's `Samples~` aren't visible in the project. To open or edit them, rename the package's `Samples~` folder to `Samples` (Unity then imports it in place, with the existing `.meta` files, so every GUID is preserved). When you're done, rename it back, and delete the generated `Samples.meta`. Do this *before committing*, not just before pushing: while the folder is renamed, `git status` shows the whole `Samples~` tree as deleted, plus an untracked `Samples/`. Alternatively, import a sample from the package's **Samples** tab in the Package Manager, which copies it into `Assets/Samples/`. Changes made there don't flow back to the package.

Don't use both at once: delete any imported copy under `Assets/Samples/` before renaming `Samples~` in place. Otherwise the project holds two copies of every sample assembly definition, which is a compile error, and two copies of every asset GUID, which Unity resolves by giving one copy new GUIDs.

## Releasing

Each package's UPM branch holds only that package's folder, as a `git subtree split` of it. Publishing is automated: [`publish-upm-package.yml`](.github/workflows/publish-upm-package.yml) is a reusable workflow holding the logic, and each package has a small caller that sets its folder, UPM branch and tag prefix, e.g. [`publish-upm-propulsion.yml`](.github/workflows/publish-upm-propulsion.yml). Whenever `main` has a package version with no release tag yet, the workflow updates that package's UPM branch and tags it, provided the version's `CHANGELOG.md` entry is dated.

To release a package:

1. Bump `version` in its `package.json`, and add a `## [X.Y.Z] - Unreleased` entry to its `CHANGELOG.md`.
2. Before or after merging to `main`, replace `Unreleased` with the release date (`YYYY-MM-DD`). The workflow publishes as soon as `main` has a dated entry. Until then it skips with a warning.

To add a package: create `Packages/<package-name>/`, add it to `testables` in `Packages/manifest.json`, and add a caller workflow next to `publish-upm-propulsion.yml` with its own folder, UPM branch (e.g. `upm-core`) and tag prefix (e.g. `core-v`), so its tags can't collide with another package's.

To publish by hand instead (e.g. if the workflow is unavailable), run on an up-to-date `main`. For `com.yohash.propulsion`:
```
git subtree split --prefix=Packages/com.yohash.propulsion --branch upm
git push origin upm
git tag -a vX.Y.Z upm -m "Propulsion Toolkit X.Y.Z"
git push origin vX.Y.Z
```

### Rebuilding a UPM branch

A package's UPM branch must only ever fast-forward, and the workflow refuses to force-push. Moving a package's folder, as in the move from `Assets/Propulsion` to `Packages/com.yohash.propulsion`, changes the split's history, so the branch has to be rebuilt once, deliberately. On an up-to-date `main`:
```
git subtree split --prefix=Packages/com.yohash.propulsion --branch upm-rebuilt
git push --force origin upm-rebuilt:upm
git branch -D upm-rebuilt
```
Existing release tags keep pointing at their original commits, so installs pinned to a tag (e.g. `#v0.6.0`) keep working. Only installs of the branch itself (`#upm`) move to the new history.
