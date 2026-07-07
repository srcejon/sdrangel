<h1>MeshCore modulator plugin</h1>

<h2>Introduction</h2>

This plugin codes and modulates a transmission signal based the LoRa Chirp
Spread Spectrum (CSS) modulation scheme with a MeshCore payload.

MeshCore (https://github.com/meshcore-dev/MeshCore, MIT) is a mesh networking
protocol distinct from Meshtastic. The two share LoRa as the PHY layer but
differ at the packet layer (envelope, addressing, crypto, routing). This
plugin builds and transmits MeshCore wire packets — ADVERT, TXT_MSG,
ANON_REQ, GRP_TXT, ACK — using vendored crypto (Monocypher for Ed25519 +
X25519, tiny-AES-c for AES-128-ECB, Qt SHA-256 + hand-rolled HMAC-SHA256).

The encode pipeline is derived from the Meshtastic modulator
(LoRa-SDR-based, configurable BW/SF/CR/sync). MeshCore EU defaults
are baked in: **869.618 MHz / 62.5 kHz / SF 8 / CR 4/8**, sync word
0x12.

This is a companion to the MeshCore demodulator plugin
(`channelrx/demodmeshcore`) and the standalone `lora_trx` headless
transceiver in gr4-lora.

<h2>Status</h2>

Functional surfaces:
- TX of ADVERT / TXT_MSG / ANON_REQ / GRP_TXT / ACK / PATH / CTRL
  via the `MESHCORE:` command syntax below.
- LoRa PHY with MeshCore EU radio defaults.
- Identity store: an Ed25519 seed is persisted under the application
  support directory, auto-loaded by `MeshcoreMod`, and used to sign
  outgoing ADVERTs. Encoder output is bit-exact against gr4-lora's
  `meshcore_tx.py::build_advert_raw` reference for the same seed and
  timestamp.
- REST sendNow action (`SWGMeshcoreModActions`) for programmatic
  single-packet TX.
- Radio parameters (BW, SF, CR, preamble length, frequency offset)
  exposed via combo controls. BW combo includes 62500 / 125000 /
  250000 Hz — the bandwidths used by the MeshCore firmware CLI's
  `set radio freq_MHz,bw_kHz,sf,cr` command.
- Preset combo populated with the well-known MeshCore regional
  defaults (EU_NARROW, EU_LONG_RANGE, EU_MEDIUM_RANGE, AU,
  AU_VICTORIA, CZ_NARROW, EU_433_LONG_RANGE, NZ, NZ_NARROW, PT_433,
  PT_868, CH, USA, VN, USER). Default is `EU_NARROW` (869.618 MHz /
  SF 8 / BW 62.5 kHz / CR 4/8). Selecting a preset applies its
  freq/BW/SF/CR via `modemmeshcore::command::applyMeshcorePreset`.
- The Region and Channel controls are hidden: region is implicit
  in the preset frequency, and MeshCore has no numbered Channel
  concept.

Outstanding:
- WebAPI schema now uses dedicated `SWGMeshcoreModSettings` /
  `SWGMeshcoreModReport` / `SWGMeshcoreModActions` schemas (no longer
  a placeholder).

<h2>MESHCORE: command syntax</h2>

Send a MeshCore wire packet by setting the channel's "Text message" to a
line starting with `MESHCORE:`:

```
MESHCORE: type=advert; seed=<hex64>; name=NodeA
MESHCORE: type=txt_msg; seed=<hex64>; dest=<hex64>; text=Hello
MESHCORE: type=anon_req; seed=<hex64>; dest=<hex64>; data=<hex>
MESHCORE: type=grp_txt; channel=public; text=Hello group
MESHCORE: type=ack; dest=<hex64>; msg_hash=<hex8>
```

Optional radio overrides on any line: `sf=8`, `bw=62500`, `cr=8` (for 4/8),
`sync=0x12`, `freq=869.618M`, `preamble=8`.

<h2>References</h2>

- MeshCore protocol: https://github.com/meshcore-dev/MeshCore
- gr4-lora python implementation: scripts/src/lora/core/meshcore_crypto.py
- Vendored Monocypher: https://monocypher.org (BSD-2-Clause OR CC0-1.0)
- Vendored tiny-AES-c: https://github.com/kokke/tiny-AES-c (public domain)
