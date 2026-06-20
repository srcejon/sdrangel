///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// This program is distributed in the hope that it will be useful,               //
// but WITHOUT ANY WARRANTY; without even the implied warranty of                //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                  //
// GNU General Public License V3 for more details.                               //
//                                                                               //
// You should have received a copy of the GNU General Public License             //
// along with this program. If not, see <http://www.gnu.org/licenses/>.          //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_FEATURE_CAMERAAUDIOBYTEQUEUE_H_
#define INCLUDE_FEATURE_CAMERAAUDIOBYTEQUEUE_H_

#include <algorithm>

#include <QByteArray>

// FIFO byte buffer for the stream / decoder audio path. Consuming from the front
// advances a read offset (O(1)) instead of memmoving the remainder on every take
// — QByteArray::remove(0, n) is an O(n) memmove, and these buffers are drained
// once per present tick / decoded frame under a lock. The live region
// [m_head, end) stays contiguous, so callers can still reinterpret_cast
// constData() for the resampler's inter-sample interpolation. The front shift is
// amortized: a single compaction memmove happens only once the dead prefix grows
// at least as large as the live data, which bounds the backing storage to ~2x the
// live bytes and shifts each byte at most once per drain cycle.
//
// Not internally synchronized — callers hold their existing audio mutex (the
// stream-audio mutex / the decoder pending-audio mutex), exactly as for the
// QByteArray it replaces.
class CameraAudioByteQueue
{
public:
    [[nodiscard]] int size() const { return static_cast<int>(m_buffer.size()) - m_head; }
    [[nodiscard]] bool isEmpty() const { return m_head >= static_cast<int>(m_buffer.size()); }

    // Start of the live region (contiguous, size() bytes). Stays 4-byte aligned as
    // long as appends/consumes are frame-aligned (they are), so the qint16 reads
    // in the resampler are aligned.
    [[nodiscard]] const char* constData() const { return m_buffer.constData() + m_head; }

    void append(const char* data, int len)
    {
        if (len <= 0) {
            return;
        }
        compactIfNeeded();
        m_buffer.append(data, len);
    }

    void append(const QByteArray& data) { append(data.constData(), static_cast<int>(data.size())); }

    // Drop n bytes from the front (clamped). O(1) amortized.
    void consume(int n)
    {
        m_head += std::min(std::max(0, n), size());
        if (m_head >= static_cast<int>(m_buffer.size())) {
            // Fully drained — reset cheaply (no memmove) so a buffer that empties
            // each cycle never compacts at all.
            m_buffer.clear();
            m_head = 0;
        }
    }

    // Copy the first n (clamped) live bytes out.
    [[nodiscard]] QByteArray left(int n) const { return m_buffer.mid(m_head, std::min(std::max(0, n), size())); }

    void clear()
    {
        m_buffer.clear();
        m_head = 0;
    }

private:
    void compactIfNeeded()
    {
        // Reclaim the dead prefix once it is at least as large as the live data
        // (and non-trivial in absolute terms, to avoid churning on tiny buffers).
        if ((m_head > 0) && (m_head >= size()) && (m_head >= 4096)) {
            m_buffer.remove(0, m_head);
            m_head = 0;
        }
    }

    QByteArray m_buffer;
    int m_head = 0;
};

#endif // INCLUDE_FEATURE_CAMERAAUDIOBYTEQUEUE_H_
