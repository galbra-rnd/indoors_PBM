# indoors_PBM

The Policy Bordering Module (A.K.A the Pink Box Module)

## Running the BPM is as easy as

```shell-command
roslaunch indoors_pbm pbm.launch
```

### After launching the PBM

One need to load a policy:

```shell-command
rostopic pub /PBM/in/load_policy/<DESIRED_POLICY_NUMBER> std_msgs/Empty
```

## Preparing Policy

Policies reside inside `indoors_PBM/config/policies`. Each policy must be inside a folder with name: `p#`.

Within each folder *one* will find:  

- Policy.md - A policy description.
- Policy.xml - A behavior tree representation.
- Policy.yaml - A policy thresholds.
