///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Tom Hensel <code@jitter.eu>                                //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// MeshCore crypto primitives. Wraps vendored Monocypher (Ed25519 + X25519 +     //
// SHA-512) and tiny-AES (AES-128 ECB) behind a Qt-friendly QByteArray API.      //
///////////////////////////////////////////////////////////////////////////////////

#include "meshcore_crypto.h"
#include "meshcorepacket.h"
#include "tiny_aes.h"

#include "monocypher.h"
#include "monocypher-ed25519.h"

#include <QCryptographicHash>

#include <algorithm>
#include <cstring>

namespace modemmeshcore
{
namespace detail
{

// ---- scalar clamping ----------------------------------------------------------

QByteArray clampScalar(const QByteArray& scalar)
{
    if (scalar.size() != kPubKeySize) {
        return QByteArray();
    }

    QByteArray out(scalar);
    auto* p = reinterpret_cast<uint8_t*>(out.data());
    p[0]  &= 248;
    p[31] &= 63;
    p[31] |= 64;
    return out;
}

// ---- expanded key (SHA-512(seed) + clamp) -------------------------------------

QByteArray expandedKey(const QByteArray& seed)
{
    if (seed.size() != kPubKeySize) {
        return QByteArray();
    }

    uint8_t hash[64];
    crypto_sha512(hash, reinterpret_cast<const uint8_t*>(seed.constData()),
                  static_cast<size_t>(seed.size()));

    // Clamp the first 32 bytes (becomes the Ed25519 scalar).
    hash[0]  &= 248;
    hash[31] &= 63;
    hash[31] |= 64;

    return QByteArray(reinterpret_cast<const char*>(hash), 64);
}

// ---- public key derivation ----------------------------------------------------
//
// Note: Monocypher's crypto_ed25519_key_pair produces a 64-byte secret_key
// in its own internal layout (NOT MeshCore's expandedKey form). We use it
// here only to obtain the public key — the secret_key is discarded.

QByteArray derivePubKey(const QByteArray& seed)
{
    if (seed.size() != kPubKeySize) {
        return QByteArray();
    }

    QByteArray seedCopy(seed); // crypto_ed25519_key_pair wipes the seed buffer
    uint8_t mcSecretKey[64];
    QByteArray pub32(kPubKeySize, '\0');
    crypto_ed25519_key_pair(mcSecretKey,
                            reinterpret_cast<uint8_t*>(pub32.data()),
                            reinterpret_cast<uint8_t*>(seedCopy.data()));
    return pub32;
}

// ---- ECDH shared secret -------------------------------------------------------

QByteArray sharedSecret(const QByteArray& seed, const QByteArray& otherPub32)
{
    if (seed.size() != kPubKeySize || otherPub32.size() != kPubKeySize) {
        return QByteArray();
    }

    // Build MeshCore's expanded private key and take its first 32 bytes as the
    // X25519 scalar. expandedKey() handles SHA-512 + clamp.
    const QByteArray expanded = expandedKey(seed);
    if (expanded.size() != kPrvKeySize) {
        return QByteArray();
    }

    // Convert the peer's Ed25519 public key (Edwards form) into its X25519
    // counterpart (Montgomery form). Pure curve map, hash-independent.
    uint8_t curvePk[32];
    crypto_eddsa_to_x25519(curvePk,
                           reinterpret_cast<const uint8_t*>(otherPub32.constData()));

    uint8_t shared[32];
    crypto_x25519(shared,
                  reinterpret_cast<const uint8_t*>(expanded.constData()),
                  curvePk);
    return QByteArray(reinterpret_cast<const char*>(shared), 32);
}

// ---- HMAC-SHA256 (RFC 2104) over Qt's QCryptographicHash::Sha256 --------------

QByteArray hmacSha256(const QByteArray& key, const QByteArray& data)
{
    constexpr int blockSize = 64; // SHA-256 block size

    QByteArray k(key);
    if (k.size() > blockSize) {
        k = QCryptographicHash::hash(k, QCryptographicHash::Sha256);
    }
    if (k.size() < blockSize) {
        k.append(blockSize - k.size(), '\0');
    }

    QByteArray innerPad(blockSize, '\0');
    QByteArray outerPad(blockSize, '\0');
    for (int i = 0; i < blockSize; ++i) {
        innerPad[i] = static_cast<char>(static_cast<uint8_t>(k[i]) ^ 0x36);
        outerPad[i] = static_cast<char>(static_cast<uint8_t>(k[i]) ^ 0x5C);
    }

    QCryptographicHash inner(QCryptographicHash::Sha256);
    inner.addData(innerPad);
    inner.addData(data);

    QCryptographicHash outer(QCryptographicHash::Sha256);
    outer.addData(outerPad);
    outer.addData(inner.result());
    return outer.result();
}

// ---- encrypt-then-MAC ---------------------------------------------------------

QByteArray encryptThenMac(const QByteArray& shared, const QByteArray& plaintext)
{
    if (shared.size() < kPubKeySize) {
        return QByteArray();
    }

    AesCtx aes;
    if (!aes.init(reinterpret_cast<const uint8_t*>(shared.constData()), kCipherKeySize)) {
        return QByteArray();
    }

    // Pad plaintext to a 16-byte boundary with 0x00.
    QByteArray padded(plaintext);
    const int pad = (kCipherBlockSize - padded.size() % kCipherBlockSize) % kCipherBlockSize;
    if (pad > 0) {
        padded.append(pad, '\0');
    }

    // ECB block-by-block.
    QByteArray ciphertext(padded.size(), '\0');
    for (int off = 0; off < padded.size(); off += kCipherBlockSize) {
        aes.encryptBlock(reinterpret_cast<const uint8_t*>(padded.constData() + off),
                         reinterpret_cast<uint8_t*>(ciphertext.data() + off));
    }

    // 2-byte truncated HMAC over ciphertext.
    QByteArray macKey = shared.left(kPubKeySize);
    QByteArray mac = hmacSha256(macKey, ciphertext).left(kCipherMacSize);

    QByteArray out;
    out.reserve(mac.size() + ciphertext.size());
    out.append(mac);
    out.append(ciphertext);
    return out;
}

// ---- MAC-then-decrypt ---------------------------------------------------------

QByteArray macThenDecrypt(const QByteArray& shared, const QByteArray& macThenCipher)
{
    if (shared.size() < kPubKeySize) {
        return QByteArray();
    }
    if (macThenCipher.size() < kCipherMacSize + kCipherBlockSize) {
        return QByteArray();
    }
    if ((macThenCipher.size() - kCipherMacSize) % kCipherBlockSize != 0) {
        return QByteArray(); // ciphertext must be a whole number of blocks
    }

    QByteArray macReceived = macThenCipher.left(kCipherMacSize);
    QByteArray ciphertext = macThenCipher.mid(kCipherMacSize);

    QByteArray macKey = shared.left(kPubKeySize);
    QByteArray macComputed = hmacSha256(macKey, ciphertext).left(kCipherMacSize);
    if (macReceived != macComputed) {
        return QByteArray(); // MAC mismatch
    }

    AesCtx aes;
    if (!aes.init(reinterpret_cast<const uint8_t*>(shared.constData()), kCipherKeySize)) {
        return QByteArray();
    }

    QByteArray plaintext(ciphertext.size(), '\0');
    for (int off = 0; off < ciphertext.size(); off += kCipherBlockSize) {
        aes.decryptBlock(reinterpret_cast<const uint8_t*>(ciphertext.constData() + off),
                         reinterpret_cast<uint8_t*>(plaintext.data() + off));
    }
    return plaintext;
}

// ---- Ed25519 sign / verify ----------------------------------------------------

QByteArray signEd25519(const QByteArray& seed, const QByteArray& message)
{
    if (seed.size() != kPubKeySize) {
        return QByteArray();
    }

    // Materialize Monocypher's 64-byte secret_key from the seed.
    // (Discard the public key it computes — caller already has it via
    // derivePubKey if needed.)
    QByteArray seedCopy(seed); // crypto_ed25519_key_pair wipes the seed buffer
    uint8_t mcSecretKey[64];
    uint8_t mcPub[32];
    crypto_ed25519_key_pair(mcSecretKey,
                            mcPub,
                            reinterpret_cast<uint8_t*>(seedCopy.data()));

    QByteArray signature(kSignatureSize, '\0');
    crypto_ed25519_sign(reinterpret_cast<uint8_t*>(signature.data()),
                        mcSecretKey,
                        reinterpret_cast<const uint8_t*>(message.constData()),
                        static_cast<size_t>(message.size()));
    return signature;
}

bool verifyEd25519(const QByteArray& signature, const QByteArray& pub32, const QByteArray& message)
{
    if (signature.size() != kSignatureSize || pub32.size() != kPubKeySize) {
        return false;
    }
    const int rc = crypto_ed25519_check(
        reinterpret_cast<const uint8_t*>(signature.constData()),
        reinterpret_cast<const uint8_t*>(pub32.constData()),
        reinterpret_cast<const uint8_t*>(message.constData()),
        static_cast<size_t>(message.size()));
    return rc == 0;
}

} // namespace detail

// ---- public protocol helpers --------------------------------------------------

PathLen decodePathLen(uint8_t raw)
{
    PathLen p;
    p.hashCount = raw & kPathCountMask;
    p.hashSize = ((raw >> kPathModeShift) & 0x03) + 1;
    p.totalBytes = p.hashCount * p.hashSize;
    return p;
}

uint8_t encodePathLen(int hashCount, int hashSize)
{
    if (hashCount < 0 || hashCount > 63 || hashSize < 1 || hashSize > 3) {
        return 0; // invalid -> empty path
    }
    return static_cast<uint8_t>(((hashSize - 1) << kPathModeShift) | (hashCount & kPathCountMask));
}

} // namespace modemmeshcore
