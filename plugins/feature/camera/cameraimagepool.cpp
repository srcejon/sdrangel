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

#include "cameraimagepool.h"

#include <atomic>
#include <cstdlib>
#include <unordered_map>
#include <vector>

#include <QMutex>
#include <QMutexLocker>

// Shared, heap-allocated free list. Held alive by an intrusive refcount: one ref
// for the owning CameraImagePool plus one per outstanding image. Deleted only
// when the last ref drops, so a buffer release that fires after the pool/decoder
// is gone (e.g. the GUI still showing the last frame) never touches freed state.
struct CameraImagePoolState
{
    QMutex mutex;
    // Free buffers bucketed by byte size. Only the current size is recycled;
    // other-size buckets are dropped on a geometry/format change so memory does
    // not accumulate across resolution changes.
    std::unordered_map<std::size_t, std::vector<uchar*>> freeBuffers;
    std::size_t currentBufSize = 0;
    int maxPerBucket = 8;
    std::atomic<int> refCount { 1 };

    ~CameraImagePoolState()
    {
        for (auto& bucket : freeBuffers)
        {
            for (uchar *buffer : bucket.second) {
                std::free(buffer);
            }
        }
    }
};

namespace {

// Per-outstanding-image record threaded through QImage's cleanupInfo. Tiny (a
// few words) versus the multi-MB buffer it tracks.
struct CameraImagePoolHandle
{
    CameraImagePoolState *state;
    uchar *buffer;
    std::size_t bufSize;
};

}

CameraImagePool::CameraImagePool(int maxBuffersPerSize) :
    m_state(new CameraImagePoolState())
{
    m_state->maxPerBucket = qMax(1, maxBuffersPerSize);
}

CameraImagePool::~CameraImagePool()
{
    // Drop the pool's own ref; outstanding images keep the state alive until they
    // are all destroyed.
    if (m_state->refCount.fetch_sub(1, std::memory_order_acq_rel) == 1) {
        delete m_state;
    }
    m_state = nullptr;
}

int CameraImagePool::bytesPerPixelForFormat(QImage::Format format)
{
    switch (format)
    {
    case QImage::Format_Grayscale8:
        return 1;
    case QImage::Format_RGB16:
    case QImage::Format_Grayscale16:
        return 2;
    case QImage::Format_RGB888:
        return 3;
    case QImage::Format_RGB32:
    case QImage::Format_ARGB32:
    case QImage::Format_ARGB32_Premultiplied:
    case QImage::Format_RGBX8888:
    case QImage::Format_RGBA8888:
    case QImage::Format_RGBA8888_Premultiplied:
        return 4;
    case QImage::Format_RGBA64:
    case QImage::Format_RGBX64:
    case QImage::Format_RGBA64_Premultiplied:
        return 8;
    default:
        return 0;
    }
}

QImage CameraImagePool::acquire(int width, int height, QImage::Format format)
{
    const int bytesPerPixel = bytesPerPixelForFormat(format);
    if ((width <= 0) || (height <= 0) || (bytesPerPixel <= 0)) {
        // Unmodelled format or degenerate size: hand back a plain image so the
        // caller still works, just without pooling.
        return QImage(width, height, format);
    }

    // Match QImage's own 32-bit-aligned scanline stride so the buffer is exactly
    // what an internally allocated QImage of this geometry would use; downstream
    // reads image.bytesPerLine(), so this stride is authoritative either way.
    const int depthBits = bytesPerPixel * 8;
    const int bytesPerLine = ((width * depthBits + 31) >> 5) << 2;
    const std::size_t bufSize = static_cast<std::size_t>(bytesPerLine) * static_cast<std::size_t>(height);

    uchar *buffer = nullptr;
    {
        QMutexLocker locker(&m_state->mutex);
        if (bufSize != m_state->currentBufSize)
        {
            // Geometry/format changed: free stale-size buffers so they don't
            // linger after a resolution change.
            for (auto it = m_state->freeBuffers.begin(); it != m_state->freeBuffers.end(); )
            {
                if (it->first != bufSize)
                {
                    for (uchar *stale : it->second) {
                        std::free(stale);
                    }
                    it = m_state->freeBuffers.erase(it);
                }
                else
                {
                    ++it;
                }
            }
            m_state->currentBufSize = bufSize;
        }
        auto& bucket = m_state->freeBuffers[bufSize];
        if (!bucket.empty())
        {
            buffer = bucket.back();
            bucket.pop_back();
        }
    }

    if (!buffer)
    {
        buffer = static_cast<uchar*>(std::malloc(bufSize));
        if (!buffer) {
            return QImage();
        }
    }

    // One outstanding-image ref. The pool itself always holds a ref while alive,
    // so the count can never reach zero here.
    m_state->refCount.fetch_add(1, std::memory_order_relaxed);
    auto *handle = new CameraImagePoolHandle{ m_state, buffer, bufSize };
    return QImage(buffer, width, height, bytesPerLine, format,
        &CameraImagePool::releaseTrampoline, handle);
}

void CameraImagePool::releaseTrampoline(void *info)
{
    auto *handle = static_cast<CameraImagePoolHandle*>(info);
    CameraImagePoolState *state = handle->state;

    {
        QMutexLocker locker(&state->mutex);
        // Recycle only buffers of the current size, and only up to the cap;
        // anything else is freed so the pool can't grow without bound or hoard
        // stale-size buffers after a resolution change.
        if ((handle->bufSize == state->currentBufSize)
            && (static_cast<int>(state->freeBuffers[handle->bufSize].size()) < state->maxPerBucket))
        {
            state->freeBuffers[handle->bufSize].push_back(handle->buffer);
            handle->buffer = nullptr;
        }
    }

    if (handle->buffer) {
        std::free(handle->buffer);
    }
    delete handle;

    // Drop this image's ref; the last ref to die (which, after the pool is gone,
    // could be this one, on any thread) deletes the shared state.
    if (state->refCount.fetch_sub(1, std::memory_order_acq_rel) == 1) {
        delete state;
    }
}

void CameraImagePool::clear()
{
    QMutexLocker locker(&m_state->mutex);
    for (auto& bucket : m_state->freeBuffers)
    {
        for (uchar *buffer : bucket.second) {
            std::free(buffer);
        }
    }
    m_state->freeBuffers.clear();
    m_state->currentBufSize = 0;
}
