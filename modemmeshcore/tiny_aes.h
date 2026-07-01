///////////////////////////////////////////////////////////////////////////////////
// Tiny AES adapted from tiny-AES-c (public domain / unlicense).                 //
// Original: https://github.com/kokke/tiny-AES-c                                 //
// This file imports only the AES-128 (and AES-256) block primitive used by      //
// MeshCore's AES-128-ECB mode. CTR / CBC wrappers are intentionally omitted —   //
// MeshCore does ECB only, on payload boundaries.                                //
///////////////////////////////////////////////////////////////////////////////////

#ifndef MODEMMESHCORE_TINY_AES_H_
#define MODEMMESHCORE_TINY_AES_H_

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

namespace modemmeshcore
{

// Single-block AES context. Supports AES-128 (16-byte key) and AES-256 (32-byte key).
// MeshCore uses AES-128 only.
class AesCtx
{
public:
    bool init(const uint8_t* key, std::size_t keyLen)
    {
        if (keyLen != 16 && keyLen != 32) {
            return false;
        }

        m_nk = static_cast<int>(keyLen) / 4;
        m_nr = m_nk + 6;
        const int words = 4 * (m_nr + 1);
        m_roundKey.resize(words * 4);
        keyExpansion(key);
        return true;
    }

    void encryptBlock(const uint8_t in[16], uint8_t out[16]) const
    {
        std::array<uint8_t, 16> state;
        std::memcpy(state.data(), in, 16);

        addRoundKey(state.data(), 0);

        for (int round = 1; round < m_nr; ++round)
        {
            subBytes(state.data());
            shiftRows(state.data());
            mixColumns(state.data());
            addRoundKey(state.data(), round);
        }

        subBytes(state.data());
        shiftRows(state.data());
        addRoundKey(state.data(), m_nr);

        std::memcpy(out, state.data(), 16);
    }

    void decryptBlock(const uint8_t in[16], uint8_t out[16]) const
    {
        std::array<uint8_t, 16> state;
        std::memcpy(state.data(), in, 16);

        addRoundKey(state.data(), m_nr);

        for (int round = m_nr - 1; round >= 1; --round)
        {
            invShiftRows(state.data());
            invSubBytes(state.data());
            addRoundKey(state.data(), round);
            invMixColumns(state.data());
        }

        invShiftRows(state.data());
        invSubBytes(state.data());
        addRoundKey(state.data(), 0);

        std::memcpy(out, state.data(), 16);
    }

private:
    int m_nk = 0;
    int m_nr = 0;
    std::vector<uint8_t> m_roundKey;

    static uint8_t xtime(uint8_t x)
    {
        return static_cast<uint8_t>((x << 1) ^ (((x >> 7) & 1) * 0x1B));
    }

    static uint8_t mul(uint8_t a, uint8_t b)
    {
        uint8_t res = 0;
        uint8_t x = a;
        uint8_t y = b;

        while (y)
        {
            if (y & 1) {
                res ^= x;
            }
            x = xtime(x);
            y >>= 1;
        }

        return res;
    }

    static uint8_t sub(uint8_t x)
    {
        static const std::array<uint8_t, 256> sbox = {
            0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
            0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
            0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
            0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
            0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
            0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
            0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
            0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
            0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
            0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
            0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
            0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
            0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
            0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
            0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
            0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16
        };
        return sbox[x];
    }

    static void subBytes(uint8_t state[16])
    {
        for (int i = 0; i < 16; ++i) {
            state[i] = sub(state[i]);
        }
    }

    static void shiftRows(uint8_t state[16])
    {
        uint8_t t;

        t = state[1];
        state[1] = state[5];
        state[5] = state[9];
        state[9] = state[13];
        state[13] = t;

        t = state[2];
        state[2] = state[10];
        state[10] = t;
        t = state[6];
        state[6] = state[14];
        state[14] = t;

        t = state[3];
        state[3] = state[15];
        state[15] = state[11];
        state[11] = state[7];
        state[7] = t;
    }

    static void mixColumns(uint8_t state[16])
    {
        for (int c = 0; c < 4; ++c)
        {
            uint8_t* col = &state[c * 4];
            const uint8_t a0 = col[0];
            const uint8_t a1 = col[1];
            const uint8_t a2 = col[2];
            const uint8_t a3 = col[3];

            col[0] = static_cast<uint8_t>(mul(a0, 2) ^ mul(a1, 3) ^ a2 ^ a3);
            col[1] = static_cast<uint8_t>(a0 ^ mul(a1, 2) ^ mul(a2, 3) ^ a3);
            col[2] = static_cast<uint8_t>(a0 ^ a1 ^ mul(a2, 2) ^ mul(a3, 3));
            col[3] = static_cast<uint8_t>(mul(a0, 3) ^ a1 ^ a2 ^ mul(a3, 2));
        }
    }

    void addRoundKey(uint8_t state[16], int round) const
    {
        const uint8_t* rk = &m_roundKey[round * 16];
        for (int i = 0; i < 16; ++i) {
            state[i] ^= rk[i];
        }
    }

    static uint8_t invSub(uint8_t x)
    {
        static const std::array<uint8_t, 256> rsbox = {
            0x52,0x09,0x6a,0xd5,0x30,0x36,0xa5,0x38,0xbf,0x40,0xa3,0x9e,0x81,0xf3,0xd7,0xfb,
            0x7c,0xe3,0x39,0x82,0x9b,0x2f,0xff,0x87,0x34,0x8e,0x43,0x44,0xc4,0xde,0xe9,0xcb,
            0x54,0x7b,0x94,0x32,0xa6,0xc2,0x23,0x3d,0xee,0x4c,0x95,0x0b,0x42,0xfa,0xc3,0x4e,
            0x08,0x2e,0xa1,0x66,0x28,0xd9,0x24,0xb2,0x76,0x5b,0xa2,0x49,0x6d,0x8b,0xd1,0x25,
            0x72,0xf8,0xf6,0x64,0x86,0x68,0x98,0x16,0xd4,0xa4,0x5c,0xcc,0x5d,0x65,0xb6,0x92,
            0x6c,0x70,0x48,0x50,0xfd,0xed,0xb9,0xda,0x5e,0x15,0x46,0x57,0xa7,0x8d,0x9d,0x84,
            0x90,0xd8,0xab,0x00,0x8c,0xbc,0xd3,0x0a,0xf7,0xe4,0x58,0x05,0xb8,0xb3,0x45,0x06,
            0xd0,0x2c,0x1e,0x8f,0xca,0x3f,0x0f,0x02,0xc1,0xaf,0xbd,0x03,0x01,0x13,0x8a,0x6b,
            0x3a,0x91,0x11,0x41,0x4f,0x67,0xdc,0xea,0x97,0xf2,0xcf,0xce,0xf0,0xb4,0xe6,0x73,
            0x96,0xac,0x74,0x22,0xe7,0xad,0x35,0x85,0xe2,0xf9,0x37,0xe8,0x1c,0x75,0xdf,0x6e,
            0x47,0xf1,0x1a,0x71,0x1d,0x29,0xc5,0x89,0x6f,0xb7,0x62,0x0e,0xaa,0x18,0xbe,0x1b,
            0xfc,0x56,0x3e,0x4b,0xc6,0xd2,0x79,0x20,0x9a,0xdb,0xc0,0xfe,0x78,0xcd,0x5a,0xf4,
            0x1f,0xdd,0xa8,0x33,0x88,0x07,0xc7,0x31,0xb1,0x12,0x10,0x59,0x27,0x80,0xec,0x5f,
            0x60,0x51,0x7f,0xa9,0x19,0xb5,0x4a,0x0d,0x2d,0xe5,0x7a,0x9f,0x93,0xc9,0x9c,0xef,
            0xa0,0xe0,0x3b,0x4d,0xae,0x2a,0xf5,0xb0,0xc8,0xeb,0xbb,0x3c,0x83,0x53,0x99,0x61,
            0x17,0x2b,0x04,0x7e,0xba,0x77,0xd6,0x26,0xe1,0x69,0x14,0x63,0x55,0x21,0x0c,0x7d
        };
        return rsbox[x];
    }

    static void invSubBytes(uint8_t state[16])
    {
        for (int i = 0; i < 16; ++i) {
            state[i] = invSub(state[i]);
        }
    }

    static void invShiftRows(uint8_t state[16])
    {
        uint8_t t;

        // row 1: shift right 1
        t = state[13];
        state[13] = state[9];
        state[9]  = state[5];
        state[5]  = state[1];
        state[1]  = t;

        // row 2: shift right 2 (same as left 2)
        t = state[2];
        state[2]  = state[10];
        state[10] = t;
        t = state[6];
        state[6]  = state[14];
        state[14] = t;

        // row 3: shift right 3 (same as left 1)
        t = state[3];
        state[3]  = state[7];
        state[7]  = state[11];
        state[11] = state[15];
        state[15] = t;
    }

    static void invMixColumns(uint8_t state[16])
    {
        for (int c = 0; c < 4; ++c)
        {
            uint8_t* col = &state[c * 4];
            const uint8_t a0 = col[0];
            const uint8_t a1 = col[1];
            const uint8_t a2 = col[2];
            const uint8_t a3 = col[3];

            col[0] = static_cast<uint8_t>(mul(a0, 0x0e) ^ mul(a1, 0x0b) ^ mul(a2, 0x0d) ^ mul(a3, 0x09));
            col[1] = static_cast<uint8_t>(mul(a0, 0x09) ^ mul(a1, 0x0e) ^ mul(a2, 0x0b) ^ mul(a3, 0x0d));
            col[2] = static_cast<uint8_t>(mul(a0, 0x0d) ^ mul(a1, 0x09) ^ mul(a2, 0x0e) ^ mul(a3, 0x0b));
            col[3] = static_cast<uint8_t>(mul(a0, 0x0b) ^ mul(a1, 0x0d) ^ mul(a2, 0x09) ^ mul(a3, 0x0e));
        }
    }

    static uint32_t subWord(uint32_t w)
    {
        return (static_cast<uint32_t>(sub(static_cast<uint8_t>((w >> 24) & 0xFF))) << 24)
             | (static_cast<uint32_t>(sub(static_cast<uint8_t>((w >> 16) & 0xFF))) << 16)
             | (static_cast<uint32_t>(sub(static_cast<uint8_t>((w >> 8)  & 0xFF))) << 8)
             |  static_cast<uint32_t>(sub(static_cast<uint8_t>( w        & 0xFF)));
    }

    static uint32_t rotWord(uint32_t w)
    {
        return (w << 8) | (w >> 24);
    }

    void keyExpansion(const uint8_t* key)
    {
        static const std::array<uint8_t, 11> rcon = {
            0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x1B,0x36
        };

        const int words = 4 * (m_nr + 1);
        std::vector<uint32_t> w(words, 0);

        for (int i = 0; i < m_nk; ++i)
        {
            w[i] = (static_cast<uint32_t>(key[4 * i])     << 24)
                 | (static_cast<uint32_t>(key[4 * i + 1]) << 16)
                 | (static_cast<uint32_t>(key[4 * i + 2]) << 8)
                 |  static_cast<uint32_t>(key[4 * i + 3]);
        }

        for (int i = m_nk; i < words; ++i)
        {
            uint32_t temp = w[i - 1];

            if ((i % m_nk) == 0) {
                temp = subWord(rotWord(temp)) ^ (static_cast<uint32_t>(rcon[i / m_nk]) << 24);
            } else if (m_nk > 6 && (i % m_nk) == 4) {
                temp = subWord(temp);
            }

            w[i] = w[i - m_nk] ^ temp;
        }

        for (int i = 0; i < words; ++i)
        {
            m_roundKey[4 * i]     = static_cast<uint8_t>((w[i] >> 24) & 0xFF);
            m_roundKey[4 * i + 1] = static_cast<uint8_t>((w[i] >> 16) & 0xFF);
            m_roundKey[4 * i + 2] = static_cast<uint8_t>((w[i] >> 8)  & 0xFF);
            m_roundKey[4 * i + 3] = static_cast<uint8_t>( w[i]        & 0xFF);
        }
    }
};

} // namespace modemmeshcore

#endif // MODEMMESHCORE_TINY_AES_H_
