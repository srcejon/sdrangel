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

#ifndef INCLUDE_FEATURE_CAMERAIMAGEPOOL_H_
#define INCLUDE_FEATURE_CAMERAIMAGEPOOL_H_

#include <QImage>

// Recycling pool of QImage backing buffers, to cut the per-frame malloc + page
// fault churn of allocating a fresh image for every decoded video frame (~25 MB
// at 4K, ~6 MB at 1080p, allocated/freed at frame rate). A 4K30 source faults in
// essentially every one of the image's ~6000 pages every frame because the heap
// does not hand back warm committed pages; a warm free list avoids that.
//
// Design: acquire() hands back a QImage backed by a pool-owned buffer via the
// external-data QImage constructor, registering a cleanup callback. QImage uses
// implicit sharing, so the callback fires when the LAST copy of the image is
// destroyed — anywhere, on any thread — and returns the buffer to the free list
// with zero changes to the downstream pipeline (display/detection/recording just
// see an ordinary QImage). Because the decoded image flows far downstream and a
// held copy can outlive the pool/decoder, the shared free list lives in a heap
// CameraImagePoolState carrying an intrusive refcount (the pool's own ref plus
// one per outstanding image); the state is deleted only when the last ref drops,
// so a cleanup firing after decoder teardown is safe (no use-after-free).
//
// The class itself is plain Qt (no FFmpeg dependency) so any per-frame producer
// can own an instance; only one source is active at a time, so each producer's
// pool naturally holds buffers of that source's size/format.

struct CameraImagePoolState;

class CameraImagePool
{
public:
    // maxBuffersPerSize caps the free list per buffer size, bounding retained
    // memory to ~maxBuffersPerSize × frameBytes. Default covers the decoder's
    // in-flight frame count (≤4 stream + worker/pipeline headroom) with margin.
    explicit CameraImagePool(int maxBuffersPerSize = 8);
    ~CameraImagePool();

    CameraImagePool(const CameraImagePool&) = delete;
    CameraImagePool& operator=(const CameraImagePool&) = delete;

    // Returns a QImage of the requested geometry/format backed by a pooled (or
    // freshly allocated) buffer. The caller may write into bits() immediately
    // (refcount 1, so no detach). Falls back to a plain unpooled QImage for
    // formats whose pixel stride this pool does not model, and returns a null
    // QImage if allocation fails.
    [[nodiscard]] QImage acquire(int width, int height, QImage::Format format);

    // Release all retained free buffers (e.g. between sources). Outstanding
    // images are unaffected — they return their buffers when their last copy
    // dies, and those are freed (over cap) rather than re-pooled.
    void clear();

private:
    static void releaseTrampoline(void *info);
    static int bytesPerPixelForFormat(QImage::Format format);

    CameraImagePoolState *m_state;
};

#endif // INCLUDE_FEATURE_CAMERAIMAGEPOOL_H_
