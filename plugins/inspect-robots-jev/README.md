<div align="center">

# inspect-robots-jev

TypeSafe's **Jev** structured-decision model as an
[Inspect Robots](https://github.com/robocurve/inspect-robots) policy.

</div>

Jev reads text and picks one option from a list you give it. It cannot see
images and cannot write free text or numbers. This plugin turns a robot scene
into text Jev can act on: AprilTags on the objects give their positions, the
plugin describes where the target is relative to the gripper in plain words
("3.5 cm FORWARD, 4 cm BELOW"), offers a menu of small moves, and executes
whichever one Jev picks. Registered as the policy `jev` and the scorer
`jev_cube_in_bowl`.

> [!NOTE]
> Under construction. Design and implementation plan:
> [`plans/0076-jev-decision-policy.md`](../../plans/0076-jev-decision-policy.md).

## Install

```bash
pip install inspect-robots inspect-robots-jev
```

Jev is reached through OpenRouter's Decisions endpoint (`OPENROUTER_API_KEY`)
or TypeSafe directly (`TYPESAFE_API_KEY`). Keys are read from the environment
or a `.env` file in the working directory.
