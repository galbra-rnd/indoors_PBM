# Policy #2 - Active Energy monitoring policy

This policy has 2 missions:

- **FLY** - Is there enough battery to even fly?
  - Upon changing mission color to RED, Land()
- **GO_HOME** - Given battery left estimation, and time to home: Is it possible to return home?
  - Upon changing mission color to RED, GoHome()

## A policy is a pair of threshold and behavior

### Thresholds configuration

Can be found at the **.yaml** file

### Behavior tree configuration

Can be found at the **.xml** file
