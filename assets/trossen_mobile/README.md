# `trossen_mobile` — Trossen AI Mobile follower cell asset

The two WXAI FOLLOWER arms of the Trossen AI Mobile kit on the cart, both facing
forward (-x), side by side. The mobile base is not modeled (not controlled yet).
Point `SCENE` at `scenes/cart.yaml` — same descriptor-driven consumers as the other
cells (`pinocchio`, `mujoco-sim`, `genesis`, rerun 3D view).

## Layout

```
model/
  meshes/           WXAI arm/gripper meshes, copied from assets/trossen_stationary
  mobile_ai/
    mobile_ai.xml   THE universal model: cart deck + 2 forward-facing arms + wrist
                    cameras + cam_front (sim stand-in for the cart's USB camera)
scenes/
  cart.yaml         the scene descriptor (parts, constraints, layouts)
```

## Cell geometry (real cart)

- Both arms face forward (-x); side by side along y.
- Base-to-base distance **0.65 m** (measured 2026-07; the cart rail is adjustable —
  if it changes, edit the two `follower_*_base_link` pos y-values, ±SEP/2).
- Arm subtrees are copied verbatim from
  `assets/trossen_stationary/model/stationary_ai/stationary_ai_forward.xml`; only the
  cart deck and the camera set differ.
