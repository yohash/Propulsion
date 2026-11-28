---
name: review-pr
description: Review a Propulsion pull request end to end. Check out the PR branch, review the diff for control-law and physics correctness, package and release hygiene and documentation drift, leave inline comments, run the PR's verification steps in the live Unity Editor via the Unity CLI (compile gate, EditMode and Burst tests, samples in Play mode), commit the Unity .meta files and packages-lock.json the PR is missing, post a summary with a verification table, then watch the PR for two hours to re-verify the author's follow-up changes. Use when asked to review, verify or QA a Propulsion PR (e.g. "review PR 5", "/review-pr 5").
---

# Review & verify a Propulsion PR

Argument: a PR number or URL (`$ARGUMENTS`). The repo is `yohash/Propulsion`. `gh` may only be at `C:\Program Files\GitHub CLI\gh.exe`.

Work through the phases in order, and keep the user posted after each one.

**Ask before you post or push.** Everything posted to GitHub is outward-facing. Before Phase 3, ask the user once which of these this review may do without asking again:
- post the inline review (Phase 3) and the summary (Phase 6);
- commit and push Editor-generated files to the PR branch (Phase 5);
- post follow-ups during the watch (Phase 7).

Never approve, request changes, merge, or push anything else unless the user asks.

**Editor-generated files** are the one kind of fix the reviewer makes rather than sends back. In this repo that means:
- the Unity `.meta` files for the PR's new files and folders;
- `Packages/packages-lock.json`, when the PR changes `Packages/manifest.json` or a package's `package.json` dependencies.

Only the Editor writes these correctly: lockfile entries carry registry metadata only Unity knows, and hand-made GUIDs collide. The reviewer has a live Editor at review time, so committing them is the reviewer's job once the user has agreed. Those commits hold Editor output only, never hand edits. Every other fix goes back to the author as a finding.

## The repo in brief

Read the root `README.md` and each touched package's `README.md` before reviewing. In short:
- One development Unity project (Unity `6000.3.20f1`, see `ProjectSettings/ProjectVersion.txt`) holds several **embedded UPM packages** under `Packages/`:
  - `com.yohash.propulsion`: MonoBehaviour controllers (`StablePdController`, `PidController`, `PropulsionMath`), the vehicle types, and `Samples~`. Namespace `Yohash.Propulsion`; the vehicle types are in the global namespace.
  - `com.yohash.propulsion.core`: the Burst-compatible math core (`StablePd`, `Pid`, `MotionMath`) over Unity.Mathematics. Namespace `Yohash.Propulsion.Core`. `noEngineReferences: true`, and its only dependency is `com.unity.mathematics`.
  - Further packages (e.g. `com.yohash.propulsion.entities`, ECS backends) follow the same pattern.
- `Assets/` is empty. `Documentation/` holds reference papers, kept out of every package.
- Each package is listed under `testables` in `Packages/manifest.json`, and has its own CHANGELOG, UPM branch (`upm`, `upm-core`, …) and tag prefix (`v`, `core-v`, …).
- Publishing is automatic: when `main` has a package version with no tag yet **and a dated CHANGELOG entry**, `.github/workflows/publish-upm-package.yml` splits the package to its UPM branch and tags it.
- PR bodies usually have a **Summary**, an **Already verified (outside Unity)** section, and a **Verification** section of checkbox steps for the Editor and the Test Runner. Those steps are this skill's test cases.

## Phase 1: Get on the PR branch

1. `gh pr view <N> --json headRefName,headRefOid,baseRefName,title,body,files,additions,deletions`.
2. Compare it with `git status -sb` and `git rev-parse HEAD`. If they differ:
   - the working tree must be clean (if it isn't, stop and ask). In particular, no package's `Samples~` may be renamed to `Samples`, and there must be no `Assets/Samples/` copy (see the root README's "Working on samples");
   - `git fetch origin`, then `git checkout -b <head> --track origin/<head>`, or `git checkout <head> && git pull --ff-only` if the branch exists;
   - confirm `HEAD == headRefOid`.
3. Check that `Packages/manifest.json` on the branch still has `com.unity.pipeline`, which the Unity CLI needs to reach the Editor. If it's missing, tell the user before going on.
4. Tell the user if Unity was open on a different branch. Unity only re-imports after the branch switch once it has focus (see Phase 5). If the PR adds packages to the manifest (e.g. Entities, or a git-URL package), expect a long package resolve on first focus.

## Phase 2: Code review

1. **Read the full diff:** `git diff origin/<base>...HEAD`. Read code first, then tests, then docs. Use a scratchpad for big diffs.
2. **Callers:** for every changed public signature, removed member or new API, grep `Packages/` for users, including `Samples~/`, `Tests/` and the README code examples. Look for callers that weren't updated, stale comments, and dead code left behind.
3. **Control-law and physics correctness.** This repo's value is that its controllers are *exactly* right, so check the maths, not just the code:
   - **Units:** angles in radians and angular velocity in rad/s, except where the API documents degrees. `PidController.ComputeAngle` in the MonoBehaviour package takes degrees. Unity's 2D APIs report angular velocity in **degrees/s**: `Rigidbody2D.angularVelocity`, `LowLevelPhysics2D.PhysicsBody.angularVelocity` and so ECSPhysics2D. `Rigidbody2D.rotation` is also degrees. An unconverted 2D angular velocity makes the D term about 57× too strong.
   - **Frames:** 3D angular quantities are world space (`AddTorque`, not `AddRelativeTorque`). Inertia goes through `inertiaTensorRotation`, and the order of rotations matters.
   - **Integrator and sub-steps:** gains must be solved for the engine's real integrator and sub-step count. `subSteps = 1` for PhysX, classic `Rigidbody2D`, and anything applied as an impulse or velocity change. Use the engine's count (4 by default) for forces on Box2D v3 / ECSPhysics2D. Gains solved for the wrong count are a blocking bug at low damping.
   - **Feedforward signs:** `−gravity` (times gravity scale in 2D) cancels gravity. Feedforward is `targetAcceleration − externalAcceleration`.
   - **State sources:** read the body (`rb.position`, `rb.linearVelocity`, the centre of mass), not an interpolated `Transform`. Control runs in the fixed step (`FixedUpdate`, `FixedStepSimulationSystemGroup`).
   - **Edge cases:** angle wrap (shortest way round), the first update with no history, `dt <= 0`, teleports and resets, zero or unknown mass, sleeping bodies, `Rigidbody.maxAngularVelocity` (7 rad/s by default).
   - **Claims:** every number in a doc or comment (frequency, damping, settling time `0.75 / f` to 5%, overshoot, stability bounds) should follow from the maths or be checked by a test.
4. **Tests back the claims:**
   - tests step the controller with the integrator the engine actually uses (including sub-steps), rather than an idealised continuous model;
   - new behaviour has a test that would fail without it. Where the PR claims planted-bug or mutation results, spot-check one by reading the test;
   - tolerances, sample counts or ranges weren't loosened, or cases removed, to make something pass;
   - a new public function in the core is added to `BurstCompatibilityTests`' `Evaluate`, so it's proven to Burst-compile.
5. **Package constraints and smells:**
   - **Core:** no `UnityEngine` references, no managed types (classes, strings, arrays, delegates) in its API or hot paths, no allocations, no exceptions (invalid input gives zero gains or a 0 result instead), and no dependencies beyond `com.unity.mathematics`;
   - **Optional dependencies** (Burst, the Input System, physics or ECS packages) are only reached through an assembly that compiles conditionally (`versionDefines` + `defineConstraints`). They must not become hard `package.json` dependencies unless the PR says why;
   - `package.json` dependencies match what the asmdefs reference;
   - a package that depends on another package from this repo says so in its README, with the install order, since UPM can't resolve git-URL dependencies;
   - **Duplicated maths:** `com.yohash.propulsion` and the core carry the same controllers separately. A fix or behaviour change in one needs the same change in the other, or an explicit note that it's deferred;
   - **Making it "just work":** swallowed errors, clamps or `Mathf.Max(…, epsilon)` that hide a bad input, warnings suppressed, gains or tolerances hand-tweaked without explanation;
   - **Repo conventions:** one type per file, grouped in folders by role; two-space indentation; namespaces as above; test asmdefs `autoReferenced: false` with `UNITY_INCLUDE_TESTS`.
6. **Documentation alignment:** check the package READMEs, the root README, the CHANGELOG and the PR body against what the code really does:
   - README code examples still compile against the changed API (argument order especially: several 0.6 signatures reordered `mass` / inertia first);
   - tables (controllers, samples, sub-step guidance, the root package table and layout) are updated;
   - "not yet measured" or "best guess" statements are still honest.
7. **Release and package hygiene:**
   - **Versioning:** a breaking API change bumps the minor version while in 0.x, and the CHANGELOG gets a "Migrating from" table. A version bump comes with a `## [X.Y.Z] - Unreleased` entry. Changes without a bump go under `## [Unreleased]`;
   - **A dated CHANGELOG entry for an unpublished version publishes the package as soon as it reaches `main`.** If the PR dates one, make sure that's intended, and say so in the summary;
   - `package.json`: `unity` minimum, `documentationUrl` / `changelogUrl` / `licensesUrl` point at the package's real folder, and the `samples` list matches the `Samples~` folders;
   - **A new package needs:** its folder under `Packages/`, an entry in `testables`, a caller workflow next to `publish-upm-propulsion.yml` with its own UPM branch and a tag prefix no other package uses, a row in the root README's package table and layout, and a README, CHANGELOG, `LICENSE.md` and `Third Party Notices.md`;
   - **Workflow edits:** publishing stays `main`-only (`if: github.ref == 'refs/heads/main'`), and UPM branches only fast-forward: no force-push;
   - **Samples:** the tree is committed as `Samples~` (never `Samples/`), with no `Samples.meta` and nothing under `Assets/Samples/`;
   - **Stray files:** no `*.csproj`, `*.sln` / `*.slnx`, `Library/` or `UserSettings/` content; reference papers go in `Documentation/`, not in a package.
8. **Metas and the lockfile** are housekeeping, not findings: you commit them in Phase 5. Note what's missing now:
   - every new file and folder under `Packages/` needs its `.meta`, including those under `Samples~`. Without one, GUIDs differ per machine and assets show as missing scripts;
   - deleted files' `.meta` files are deleted too (none orphaned);
   - every GUID in the PR is unique in the repo (`git grep -h '^guid:' -- '*.meta' | sort | uniq -d`). A duplicate GUID **is** a finding: it usually means a copied file;
   - a manifest or `package.json` dependency change needs a matching `Packages/packages-lock.json` update.

   Hand-written `packages-lock.json` entries are a finding.
9. **Rank** the findings: blocking, correctness risk, docs drift, minor, nit. Only include findings you've confirmed in the code. For maths findings, give the derivation or a numeric counter-example.

## Phase 3: Inline comments

Post one review with every inline comment that maps to a line **on the RIGHT side of the diff** (the line has to be inside a diff hunk). Write the JSON to a scratchpad file and run:

```
gh api -X POST repos/yohash/Propulsion/pulls/<N>/reviews --input review.json
```

The JSON needs `commit_id` (the head SHA), `event: "COMMENT"`, a short `body`, and `comments[]` of `{path, line, side: "RIGHT", body}`. Findings outside the diff go into the Phase 6 summary.

## Phase 4: Find the verification cases

The test cases are the PR body's **Verification** checklist (often grouped as "In the Unity editor", "Tests (Test Runner → EditMode)", "Samples" and "Read before merging"). Also check the PR comments for extra steps (`gh pr view <N> --comments`). Turn them into a numbered checklist, and add:
- **the expected test counts** the PR states (e.g. "233/233"), per test assembly;
- each **"Not run" / "to confirm"** item from "Already verified (outside Unity)", such as engine API names the author guessed, or anything needing Burst, baking or Play mode;
- any cases your review suggests, e.g. a reproduction for a suspected bug, or a measurement of a claimed frequency, damping or settling time.

## Phase 5: Connect to Unity and verify

Load the `unity:unity-cli` skill. Read memory `unity-cli-bridge-gotchas` first if it exists.

**Connect:**
- Run `unity status --json`. If it fails, run `unity pipeline editors --json` and `unity editors running --json` to tell the cases apart:
  - **not running:** ask the user to open Unity;
  - **Safe Mode:** compile errors, which already fails the compile gate. Read `Editor.log` (`%LOCALAPPDATA%\Unity\Editor\Editor.log`) for `error CS`;
  - **unreachable:** Unity is unfocused, resolving packages, or has a modal open. Ask the user to click into Unity, then poll `unity status --json` in a background loop until `"success": true`.
- Report the Unity version, the port and any connection problems to the user.

**Compile gate:**
- Run `unity command script eval --file .claude/skills/review-pr/editor-state.cs --timeout 60 --result-only`. It reports:
  - `scriptCompilationFailed`, `isCompiling` and `isPlaying`;
  - whether Burst compilation is on;
  - the embedded `com.yohash.*` packages and their versions, which is the Package Manager check most PRs list;
  - every type in the loaded `yohash.*` assemblies.
- Confirm the PR's new types are listed, and types it deletes are not.
- Run `unity command console get --level error`, and check the warnings for `Yohash.Propulsion*` ones. PRs usually promise "0 errors, no `Yohash.Propulsion*` warnings".

**Commit Editor-generated files** (right after the compile gate, before you create any test assets):
1. **Confirm the Editor is on this branch's code.** After a branch switch, Unity runs the previous branch's code until it refreshes and recompiles. The gate must pass with `isCompiling == false`, the PR's new types resolving and its deleted types gone.
2. **Metas for `Samples~`:** Unity doesn't import folders ending in `~`, so it never writes their metas. If Phase 2 found missing metas under a `Samples~`, generate them while running the samples (see "Samples" below). Keep the new metas when you rename the folder back.
3. **List what Unity wrote:**
   - untracked metas: `git status --porcelain --untracked-files=all | grep '^?? .*\.meta$'`;
   - the lockfile: `git status --porcelain -- Packages/packages-lock.json`.
4. **Judge them:**
   - keep a `.meta` whose asset belongs to the PR. That means the `.meta` minus its extension is a file in `git diff --name-only --diff-filter=A origin/<base>...HEAD`, or a folder that holds one. Skip everything else, such as test leftovers, `Samples.meta`, or files from the user's own work;
   - keep lockfile changes that follow from the PR's manifest or `package.json` changes. If it changed for unrelated reasons (e.g. Unity upgraded a package on its own), revert it with `git checkout -- Packages/packages-lock.json` and tell the user.
5. **Commit by explicit path** (never `git add -A` or `git add .`), ending with the attribution trailer:
   - metas: `chore: add Unity .meta files for <short scope>`, listing them in the body;
   - the lockfile: `chore: record <packages> in packages-lock.json`, saying which entries Unity added or changed.
6. **Push** (only if the user agreed): `git push origin HEAD:<headRefName>`.
   - **Push rejected because the author pushed meanwhile:** `git pull --rebase origin <headRefName>`, re-run the compile gate, then push again.
   - **No push access** (e.g. a fork): don't push. Put the files in the summary as a blocking item instead.
7. Use the new head SHA from here on: Phase 6 reports against it, and Phase 7 takes it as the baseline. Tell the user what you committed.

Unity only writes a `.meta` for a file it imported, and resolves the lockfile only for a manifest it could load. If the PR adds a file Unity can't import, or a package that won't resolve, say so rather than leaving it silently unflagged.

**Tests:**
- Run each test assembly the PR touches, plus every assembly of a package whose runtime it changes. Use `unity command test run --mode editor --filter-type Assembly --filter <assembly> --timeout 600 --result-only`. It returns one entry per test with `fullName` and `status`. Count the `Passed`, `Failed`, `Skipped` and `Inconclusive` entries, and compare them with the PR's stated counts.
- The current assemblies are:
  - `yohash.propulsion.tests.editor`;
  - `yohash.propulsion.core.tests.editor`;
  - `yohash.propulsion.core.tests.burst`.

  New packages add their own (e.g. a `…tests.physics2d` that needs its physics package).
- **Burst:** the Burst test only proves anything with Burst compilation on (`burst.IsEnabled=True` in the editor-state script), and with `com.unity.burst` installed. Otherwise the assembly doesn't compile at all. With Burst off, an Ignored or skipped result is **not** a pass. Only toggle Burst compilation if a PR step asks you to, and restore it afterwards.
- PlayMode tests, if any: `--mode playmode --async-tests true`, then poll `unity command test status`.
- For a failure, quote the test's `message` and the first `stackTrace` frame in the summary.

**Samples (when the PR touches them, or asks for a sample to be run):**
- Samples live in `Samples~`, which Unity hides. Use the root README's in-place method: confirm `Assets/Samples/` doesn't exist, then rename the package's `Samples~` to `Samples` and refresh (`AssetDatabase.Refresh()`). Unity imports it with its existing metas.
- Open the sample scene with `EditorSceneManager.OpenScene`. Check the console for missing-script warnings and errors.
- **Always restore:** rename `Samples` back to `Samples~`, delete the generated `Samples.meta`, and refresh. Check that `git status` no longer shows the `Samples~` tree as deleted. Do this before any commit, and in Clean up even if a step failed.
- The sample materials use the Built-in pipeline's Standard shader. Magenta materials in a URP or HDRP project are expected, not a finding. Input samples (the two vehicle ones) need WASD, so drive them by script, e.g. by setting the motor's inputs, and say so in the results.

**Measuring the response in Play mode:**
- Run `unity command editor.playmode start`. It turns Run In Background on until Play mode exits, so the game keeps ticking while Unity is unfocused, and leaves PlayerSettings alone. Don't set `Application.runInBackground` yourself: in the Editor that setter also flips the PlayerSettings value.
- Entering Play mode can reload the domain, so hook `Application.logMessageReceived` and other probes after it starts, not before.
- For frame-exact checks: `unity command editor.playmode pause`, then repeat an eval that records state and calls `EditorApplication.Step()` once per physics step. Read the body (`Rigidbody`/`Rigidbody2D` position, velocity, rotation), not its `Transform`. Note `Time.fixedDeltaTime`.
- To check a claimed frequency or damping, record the error after a step change of target. Measure the time to reach 5% (compare with `0.75 / f` at damping 1), the overshoot, and whether it settles. A force-driven body under the wrong sub-step count, or an unconverted degrees/s, shows up here as ringing or no settling.
- For ECS packages: inspect entities with `World.DefaultGameObjectInjectionWorld.EntityManager` queries, using `IgnoreComponentEnabledState` to see disabled enableables. Check the system order in the `FixedStepSimulationSystemGroup` against what the PR claims.

**Running checks:**
- Always use `unity command script eval --file <path>` with C# written to a scratchpad file; inline `script eval --code` gets mangled by quoting. `--timeout` is in seconds.
- Return a string from the script. With `--result-only`, a successful eval prints just that string as a JSON literal; unescape it with `sed 's/\\r\\n/\n/g; s/\\n/\n/g'`. Python may not be available, and a failing parser loses the output of an eval that already ran. A failed eval (including a script that doesn't compile) prints the full envelope instead: read `errors[0].message`.
- Script eval is not a full C# file: `ref` locals and some declarations don't compile there. Fully qualify types, or reach optional packages by reflection (as `editor-state.cs` does for Burst).
- Read Play-mode logs with `unity command console get --tail N` (run `unity command console clear` first).
- Use `SerializedObject.FindProperty` to check what the Inspector shows (e.g. an asmdef's "No Engine References", or a controller's serialized Frequency and Damping). Call private `OnValidate` hooks through reflection to stand in for Inspector edits.
- Put any test scenes or assets in `Assets/_PRVerification/`.

For each step, record **Pass / Partial / Fail / Not run**, what you actually did, and any gap between that and the step as written. Examples: "driven by script, not WASD"; "measured over 200 steps, not watched"; "Burst test run with compilation on only". Note when the step as written can't produce its expected result: that's a finding.

**Clean up (always):**
- unhook any probes, then `unity command editor.playmode stop`;
- rename any `Samples` back to `Samples~`, and delete `Samples.meta`;
- delete `Assets/_PRVerification` and its `.meta`;
- check `unity command editor status` shows `"runInBackground": false`. If a script set it, set it back to false (both the Application and PlayerSettings values);
- restore Burst compilation if you toggled it;
- if the tree was clean at the start, `git checkout -- ProjectSettings Packages/com.yohash.*` (opening scenes and refreshes can re-serialize assets), then `AssetDatabase.Refresh()`. Don't revert the lockfile or metas you committed;
- check `git status`. An untracked `.meta` for the PR, or a lockfile change for it, means the commit step missed it: commit it the same way. Delete any untracked meta left behind by your own test assets.

## Phase 6: Summary comment

Post with `gh pr comment <N> --body-file summary.md`. Include:
- a verdict headline;
- the verification table (step | result | how it was checked / caveats), with the test counts per assembly;
- the meta and lockfile commits, if you made any (short SHAs and the files), as done housekeeping rather than findings;
- whether merging publishes anything (a dated CHANGELOG entry for an unpublished version);
- overall comments, ordered blocking → correctness risks → docs drift → minor → nits (including findings that couldn't go inline);
- a one-line recommendation.

End the comment with `🤖 Generated with [Claude Code](https://claude.com/claude-code)`.

Then tell the user what you posted (with links) and anything still open, and start Phase 7.

## Phase 7: Watch for the author's response

Authors usually answer a review within the hour, with fix commits and inline replies, and sometimes merge soon after. Keep watching the PR for **2 hours after the Phase 6 summary is posted**, so your passing criteria get checked against what actually landed rather than against the author's description of it.

**Record a baseline** in a scratchpad file (`pr<N>-watch.md`):
- the reviewed head SHA, the summary comment URL and the time it was posted;
- every open item from Phase 6 and what would satisfy it.

**Set up the watch:**
- **Push first.** If the host can wake this session on new activity on this PR (comments, reviews, pushes), use that, and keep the 2-hour limit. Don't turn on any mode that makes this session push fixes to the author's PR or answer comments there. Apart from Editor-generated files, a reviewer doesn't push (see the intro).
- **Otherwise, poll.** Schedule six one-shot checks, 20 minutes apart, the first one 20 minutes after posting:
  - use `CronCreate` with `recurring: false` and minute, hour, day and month pinned for each. One-shots end the watch on their own, with no cleanup if the session is busy. Nudge a time off `:00` / `:30` by a minute;
  - give each the prompt `PR watch <k>/6 for yohash/Propulsion#<N>: run review-pr Phase 7 "Each check". Baseline: <path to pr<N>-watch.md>`;
  - tell the user the run times `CronList` reports, and that the jobs live only in this session: they fire while it is open and idle, and are lost if it closes.

**Each check:**
1. `gh pr view <N> --json state,mergedAt,headRefOid,commits,comments,reviews` and `gh api repos/yohash/Propulsion/pulls/<N>/comments` (inline replies carry `in_reply_to_id`). Only look at what's newer than the baseline.
2. **Nothing new:** post nothing on the PR, and give the user one line at most.
3. **New replies or commits:**
   - Fetch the head with `git fetch origin pull/<N>/head:pr-<N>`. This works even after a merge deletes the branch.
   - Review only `git diff <baseline SHA> pr-<N>`, with Phase 2's checks. If a change affects things outside the original findings (as a shared helper, or maths duplicated between packages, would), check those too.
   - Judge each open item by what the code, doc or test actually shows, not by the reply.
   - Re-run Phase 5 for anything whose behaviour changed: always the compile gate, "Commit Editor-generated files" (on the PR branch, since the fetched `pr-<N>` ref can't be pushed to), and the test assemblies it touches. Then run the steps the change affects, then one spot check of behaviour it shouldn't have touched. Follow Phase 5's clean-up rules.
   - Post **one** follow-up `gh pr comment` with a table of item | agree / disagree | how it was checked, ending with the Claude Code line. Don't repost the whole summary.
   - Update the baseline SHA and the open items.
4. **Merged or closed:**
   - If the merge includes commits you haven't verified, check out the updated base (`git checkout <base> && git pull --ff-only`), verify those commits there, and post a post-merge follow-up.
   - If the merge dated a CHANGELOG entry, check the publish run (`gh run list --workflow publish-upm-<package>.yml --limit 3`) and that the tag exists. Report it to the user; don't re-run or fix the workflow yourself.
   - Then cancel the remaining checks (`CronDelete`) and delete your local `pr-<N>` ref.
5. **Every item satisfied and verified:** post the follow-up and cancel the remaining checks.

**Working tree:** the user may be working in this checkout between checks. Switch branches only if the tree is clean. Otherwise, read the delta without checking out (`git diff`, `git show`), skip the Unity re-run and the generated-file commits, and say so in the follow-up and to the user. If any new files lack metas, or the delta changes dependencies without a lockfile update, list them in the follow-up.

**When the watch ends,** whether on the sixth check or cancelled early, tell the user what changed, what you verified, and which items are still open. Leave no scheduled jobs behind.
