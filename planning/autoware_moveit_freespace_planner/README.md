# autoware_moveit_freespace_planner

Drop-in Autoware parking freespace planner that calls
`AbstractPlanningAlgorithm::makePlan` via OMPL Reeds-Shepp
(`MoveItAckermannPlanner`), with the same topics/remaps as
`autoware_freespace_planner`.

## Switch from parking launch

```bash
freespace_backend:=moveit   # or legacy (default)
```

See `tier4_planning_launch/.../parking.launch.xml`.

## Params

`config/freespace_planner.param.yaml` — includes `moveit.planner_id`,
`moveit.turning_radius` (`<0` derives from vehicle_info), simplification flags.
