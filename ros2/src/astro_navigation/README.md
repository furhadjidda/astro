# astro_navigation

Nav2 navigation launch for Astro.

## Launch

```bash
ros2 launch astro_navigation navigation2.launch.py
```

## What it does

Launches the full Nav2 stack with a pre-built map (`map/astro_map_2.yaml`) and Waffle-style navigation parameters. Opens RViz with the Nav2 default view for goal setting.

## Configuration

- Map files in `map/`
- Nav2 params in `param/`
