///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Tom Hensel <code@jitter.eu>                                //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// MeshCore crypto primitives. Internal to modemmeshcore — exposed in            //
// modemmeshcore::detail so unit tests can exercise them directly.               //
//                                                                               //
// Wire-format reference: github.com/meshcore-dev/MeshCore (MIT) and the         //
// gr4-lora python implementation under scripts/src/lora/core/meshcore_crypto.py //
///////////////////////////////////////////////////////////////////////////////////

#ifndef MODEMMESHCORE_MESHCORE_CRYPTO_H_
#define MODEMMESHCORE_MESHCORE_CRYPTO_H_

#include <QByteArray>

#include "export.h"

namespace modemmeshcore
{
namespace detail
{

// Curve25519 scalar clamping (RFC 7748 §5).
//   ba[0]  &= 248  // clear low 3 bits
//   ba[31] &= 63   // clear top 2 bits
//   ba[31] |= 64   // set bit 254
// Returns the clamped 32-byte scalar (or empty on size mismatch).
MODEMMESHCORE_API QByteArray clampScalar(const QByteArray& scalar);

// Expand a 32-byte seed into MeshCore's 64-byte private key:
//   SHA-512(seed), clamp first 32 bytes, leave second 32 bytes raw.
// Returns 64 bytes (or empty on size mismatch).
MODEMMESHCORE_API QByteArray expandedKey(const QByteArray& seed);

// Derive the 32-byte Ed25519 public key from a 32-byte seed.
MODEMMESHCORE_API QByteArray derivePubKey(const QByteArray& seed);

// Compute MeshCore ECDH shared secret via X25519 from our seed and the peer's
// Ed25519 public key. Internally builds expandedKey(seed) and uses its first
// 32 bytes (clamped scalar) for the X25519 multiply.
// Returns 32-byte shared secret on success, empty on validation failure.
MODEMMESHCORE_API QByteArray sharedSecret(const QByteArray& seed,
                                          const QByteArray& otherPub32);

// AES-128-ECB encrypt then HMAC-SHA256 MAC (truncated to 2 bytes).
//   plaintext is zero-padded to 16-byte boundary before encryption.
//   key  = sharedSecret[0..16]
//   mac  = HMAC-SHA256(sharedSecret[0..32], ciphertext)[0..2]
// Returns: mac(2) || ciphertext(N*16). Empty on input error.
MODEMMESHCORE_API QByteArray encryptThenMac(const QByteArray& sharedSecret,
                                            const QByteArray& plaintext);

// Inverse of encryptThenMac. Verifies MAC, decrypts, returns plaintext
// (still zero-padded). Empty on MAC failure or input error.
MODEMMESHCORE_API QByteArray macThenDecrypt(const QByteArray& sharedSecret,
                                            const QByteArray& macThenCipher);

// Sign a message with the MeshCore identity (32-byte seed).
// Returns 64-byte signature, empty on input-size error.
MODEMMESHCORE_API QByteArray signEd25519(const QByteArray& seed,
                                         const QByteArray& message);

// Verify Ed25519 signature.
MODEMMESHCORE_API bool verifyEd25519(const QByteArray& signature,
                                     const QByteArray& pub32,
                                     const QByteArray& message);

// Standard HMAC-SHA256 (RFC 2104) over Qt's QCryptographicHash::Sha256.
// Returns 32-byte digest. Used for ACK hashing too.
MODEMMESHCORE_API QByteArray hmacSha256(const QByteArray& key,
                                        const QByteArray& data);

} // namespace detail
} // namespace modemmeshcore

#endif // MODEMMESHCORE_MESHCORE_CRYPTO_H_
