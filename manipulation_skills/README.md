# Piper Pick Skill

`pick()` creates a stateful skill; the caller remains responsible for advancing
the environment:

```python
from manipulation_skills import PickConfig, pick

skill = pick(
    env,
    piper_profile,
    robot="left",
    object_name="doll_00002",
    config=PickConfig(lift_distance_m=0.10),
)

while not skill.done:
    step = skill.tick()
    observation, info = env.step(step.action)
```

When no `GraspCandidate` is supplied, the Piper adapter generates a
downward-diagonal side grasp aimed from the selected robot base toward the
object center. An explicit candidate is a TCP pose in the object's frame.

Run the included environment example with:

```bash
uv run python manipulation_skills/demo_piper_pick.py --viz none
```

The first version uses differential IK and segmented Cartesian motion. It does
not yet plan around arbitrary obstacles.
