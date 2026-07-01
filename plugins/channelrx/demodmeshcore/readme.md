<h1>MeshCore demodulator plugin</h1>

<h2>Introduction</h2>

This plugin demodulates and decodes incoming MeshCore wire packets. It is the
RX counterpart of the MeshCore modulator plugin (`channeltx/modmeshcore`)
and the standalone `lora_trx` headless transceiver in gr4-lora.

The decode pipeline is derived from the Meshtastic demodulator
(LoRa-SDR-based, configurable BW/SF/CR/sync). MeshCore EU defaults
are baked in: **869.618 MHz / 62.5 kHz / SF 8 / CR 4/8**, sync word
0x12.

<h2>Status</h2>

Functional surfaces:
- LoRa PHY decode covering MeshCore EU radio params (preamble, sync,
  CFO, soft demod), with the following adjustments: `m_nbSymbolsMax`
  default raised to 1023 (256 symbols are needed for a 120-byte
  ADVERT at SF=8 / CR=4/8; the prior default of 255 clipped this);
  channel filter cutoff widened to BW/1.25; interpolator upsample
  regime in `feed()`; `m_chirp` initialization aligned to the
  standard LoRa upchirp.
- `Packet::decodeFrame` (in `modemmeshcore`) for ADVERT / TXT_MSG /
  GRP_TXT / ANON_REQ / ACK / PATH / CTRL packets.
- ADVERT signature verification (Ed25519 via vendored Monocypher).
- Auto-augmented PSK for public channels.
- Optional UDP JSON sink (`sendJsonViaUDP`) for off-process
  observation.
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
  in the preset frequency, and group channels are managed via the
  keys dialog rather than a numbered Channel selector.

Outstanding:
- WebAPI schema reuses `SWGMeshtasticDemodSettings` /
  `SWGMeshtasticDemodReport` as a placeholder; a dedicated
  `SWGMeshcoreDemod*` schema would require regenerating the SWG
  bindings.

<h2>References</h2>

- MeshCore protocol: https://github.com/meshcore-dev/MeshCore
- gr4-lora python decoder: `scripts/src/lora/decoders/meshcore.py`
- gr4-lora python crypto:  `scripts/src/lora/core/meshcore_crypto.py`
- Vendored Monocypher: https://monocypher.org (BSD-2-Clause OR CC0-1.0)
- Vendored tiny-AES-c: https://github.com/kokke/tiny-AES-c (public domain)
