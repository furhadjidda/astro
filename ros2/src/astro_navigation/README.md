# astro_navigation

Nav2 navigation launch for Astro.

## Launch

```bash
ros2 launch astro_navigation2 navigation2.launch.py
```

## What it does

Launches the full Nav2 stack with a pre-built map (`map/astro_map_2.yaml`) and
model-specific navigation parameters. Opens RViz with the Nav2 default view for
goal setting. Set `ASTRO_ROBOT_MODEL=burger` on the machine running this launch
to select `burger.yaml`; the default is `waffle`. Pass `params_file:=/path/to/file.yaml`
to override the selected file.

The burger configuration starts from the waffle settings, with a provisional
0.15 m costmap radius. Assuming the center of the
[published 138 mm by 178 mm dimensions](https://docs.robotis.com/docs/systems/turtlebot3/features/)
matches the URDF's 32 mm base offset, the estimated maximum radius is about
0.135 m. Before autonomous motion, measure the assembled robot from `base_link`,
including all attachments, and adjust both costmap radii and other parameters
as needed.

## Configuration

- Map files in `map/`
- Nav2 params in `param/`
