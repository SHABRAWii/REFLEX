# Humanoid Learning - Summary

Humanoid learning is the process of training a robot or simulated body to move, balance, recover, and complete tasks using data-driven control methods.

It replaces some hand-written behavior with policies learned from motion data, simulation, rewards, and robot sensor feedback.

## Reference Value

| Point | Why it matters |
| --- | --- |
| Motion imitation | Learns movement patterns from human or animation data |
| Reinforcement learning | Improves behavior through reward-based simulation training |
| Sim-to-real transfer | Connects simulation training with hardware deployment |
| Retargeting | Adapts human or character motion to a robot body |
| Sensor feedback | Uses IMU, joint, and contact data for closed-loop control |
| Disturbance recovery | Important for balance, pushes, terrain changes, and falls |

## Why It Is a Good Reference

- Gives a high-level map of how modern humanoid control is trained.
- Useful for comparing data-driven control against classical control.
- Helps organize research into imitation, RL, retargeting, and deployment.
- Good foundation before studying specific tools like ProtoMotions or Rofunc.

## Repo Note

Humanoid learning is a useful reference area because it explains how robots can learn robust motion behavior instead of relying only on manually tuned rules.
