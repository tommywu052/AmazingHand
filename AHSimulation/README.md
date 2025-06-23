# Example control for the Pollen Robotics "AmazingHand" (WIP)

- Install uv: https://docs.astral.sh/uv/getting-started/installation/
- Install dora-rs: https://dora-rs.ai/docs/guides/Installation/installing
  - start the daemon: `dora up`
- In console run: `dora start dataflow.yml`
- In another console run the hand tracker: `uv run src/hand_tracker.py`
- In another console run the simulation: `uv run src/mj_mink_right.py`
