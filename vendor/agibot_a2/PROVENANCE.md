# Provenance — AgiBot public A2 model

- **Source:** https://github.com/Link-U-OS/rl_training, folder `resources/robots/agibot_a2/`
- **Commit:** `8bf8fd8bbcc38d09731d1b15f894ce7388b08c3d` (2026-03-04, "fix: revert accidental changes to reward function and reward scales. (#10)")
- **Downloaded:** 2026-10-22, sparse clone, with Michael's OK in chat
- **Licence:** Mulan PSL v2. Copy: `LICENSE-MulanPSL2` (the repo root `LICENSE`)
- **Contents:** 97 files, 17 MB. 1 URDF (`urdf/agibot_a2_dof12.urdf`), 1 MuJoCo file (`mjcf/agibot_a2_dof12.mjcf`),
  48 visual meshes (`meshes/public/`), 47 convex collision hulls (`meshes/convex/`). All are binary STL; none are LFS pointers
- **Integrity:** `SHA256SUMS` (98 lines = the 97 files + the licence copy). Check it from this folder:
  `shasum -a 256 -c SHA256SUMS`

Do not edit files here. Corrections for Luka go into our own model, which is derived from this one.
