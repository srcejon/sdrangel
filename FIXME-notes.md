## meshtasticdemodgui.cpp - checkbox signal pattern
All checkboxes currently use stateChanged(int) pattern.
Consider refactoring all to clicked(bool) for simplicity in a future cleanup commit.
Affects: sendViaUDP, sendJsonViaUDP, and likely other checkboxes in the same file.

## meshtasticdemod.cpp:796 - APRS guard
m_lastMsgBytes[0] is read without an explicit non-empty size check before the APRS detection block.
Pre-existing issue, not introduced by JSON UDP feature. Fix separately in its own commit.
Safe fix: add `&& !m_lastMsgBytes.isEmpty()` to the APRS condition.

## meshtasticdemod.cpp:658 - hash_matching_index fabricated
hash_matching_index is hardcoded to 0 for decrypted packets and "none"
for all others. The decoder sorts key candidates by hash match but does
not expose the actual index used in DecodeResult. The field is therefore
inaccurate for multi-key configurations. Fix requires extending
DecodeResult to expose the matched key index, then wiring it through
buildMeshtasticJsonPacket. For now the field remains but should not be
relied upon.
