# Joint-only playback recordings from the Luka iPad app

These three `.a2rec` files were copied byte-for-byte from `Documents/recordings/` in the Luka app on Michael's paired iPad Air 11-inch (M3) on 2026-10-26. The device listing supplied exact byte counts, which matched the copies; SHA-256 hashes in [`MANIFEST.json`](MANIFEST.json) were computed on the Mac because the device file API does not expose on-device hashes.

| File | Messages | Duration | Content |
| --- | ---: | ---: | --- |
| `luka-20261019-115249-legs.a2rec` | 4,921 | 167.4 s | `a2remote/joints` only |
| `luka-20261022-125322-arm.a2rec` | 6,679 | 227.3 s | `a2remote/joints` only |
| `luka-20261022-191420.a2rec` | 5,053 | 171.9 s | `a2remote/joints` only |

The Luka app's source notes say these playback files were converted from earlier recorded joint-state bags and copied to the iPad. Each message has only `t`, `j`, and `src` payload fields; the source label names its bag. No camera, LiDAR, depth, or room-map topic appears in these three files. This is a scoped content observation, not a certification of every source or timing field.

Joint units, signs, frames, and synchronization with the MuJoCo model remain unresolved. Playback motion is not evidence that the public model matches Luka's measured geometry or physical behavior. The complete Luka app data export is preserved separately in a private GitHub backup.

List the recording topics and counts without changing the files:

```sh
python3 -I tools/a2rec_info.py data/ipad-playback/recordings/*.a2rec
```
