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

#include "cameraplatesolverinternal.h"

// Solve pipeline: catalog fetch/build, seed generation, pose evaluation + matching (non-static SolverContext members, WS5).

// Track 0a: hermetic-catalog offline mode. When SDRANGEL_CAMERA_PLATE_SOLVER_OFFLINE is set the
// solver must NOT reach the network -- every Siril SPCC byte-range request that misses the
// on-disk/in-memory cache fails loudly instead of silently fetching. This turns the test corpus
// into a reproducible gate: given a warmed cache, a solve is a pure function of the frozen bytes,
// so the pollux-class nondeterminism (a silent re-fetch returning subtly different catalog data)
// can no longer perturb results. Read once per process (the harness sets the env before launch).
static bool sirilOfflineModeEnabled()
{
    static const bool offline = qEnvironmentVariableIntValue("SDRANGEL_CAMERA_PLATE_SOLVER_OFFLINE") != 0;
    return offline;
}

void CameraPlateSolver::SolverContext::clearProfileTimings()
{
    m_profileTimingOrder.clear();
    m_profileTimingStats.clear();
    m_profileMetricOrder.clear();
    m_profileMetrics.clear();
}

void CameraPlateSolver::SolverContext::recordProfileTiming(const QString& stage, qint64 elapsedMs)
{
    if (!m_profileTimingStats.contains(stage)) {
        m_profileTimingOrder.append(stage);
    }

    ProfileTiming& timing = m_profileTimingStats[stage];
    timing.totalMs += elapsedMs;
    timing.maxMs = std::max(timing.maxMs, elapsedMs);
    ++timing.count;
}

void CameraPlateSolver::SolverContext::recordProfileMetric(const QString& name, qint64 value)
{
    if (!m_profileMetrics.contains(name)) {
        m_profileMetricOrder.append(name);
    }
    m_profileMetrics[name] += value;
}

void CameraPlateSolver::SolverContext::copySearchStateFrom(const SolverContext& other)
{
    m_weakModeNormalizationPixels = other.m_weakModeNormalizationPixels;
    m_useElevationSeedPreference = other.m_useElevationSeedPreference;
    m_elevationSeedReferenceDegrees = other.m_elevationSeedReferenceDegrees;
    m_elevationSeedReferenceFovDegrees = other.m_elevationSeedReferenceFovDegrees;
    m_elevationSeedScaleDegrees = other.m_elevationSeedScaleDegrees;
    m_elevationSeedFovScaleDegrees = other.m_elevationSeedFovScaleDegrees;
    m_useAllSkyZenithPreference = other.m_useAllSkyZenithPreference;
    m_allSkyZenithReferenceElevationDegrees = other.m_allSkyZenithReferenceElevationDegrees;
    m_allSkyZenithScaleDegrees = other.m_allSkyZenithScaleDegrees;
    m_useDirectionSeedPreference = other.m_useDirectionSeedPreference;
    m_useWideCatalogMagnitudePreference = other.m_useWideCatalogMagnitudePreference;
    m_directionSeedHasRollPreference = other.m_directionSeedHasRollPreference;
    m_directionSeedReferenceAzimuthDegrees = other.m_directionSeedReferenceAzimuthDegrees;
    m_directionSeedReferenceElevationDegrees = other.m_directionSeedReferenceElevationDegrees;
    m_directionSeedReferenceRollDegrees = other.m_directionSeedReferenceRollDegrees;
    m_directionSeedReferenceFovDegrees = other.m_directionSeedReferenceFovDegrees;
    m_directionSeedAzElScaleDegrees = other.m_directionSeedAzElScaleDegrees;
    m_directionSeedRollScaleDegrees = other.m_directionSeedRollScaleDegrees;
    m_directionSeedFovScaleDegrees = other.m_directionSeedFovScaleDegrees;
    m_directionSeedMinMatchCount = other.m_directionSeedMinMatchCount;
    m_useFovSeedPreference = other.m_useFovSeedPreference;
    m_fovSeedReferenceDegrees = other.m_fovSeedReferenceDegrees;
    m_fovSeedScaleDegrees = other.m_fovSeedScaleDegrees;
    m_detectionBrightnessMetricCache = other.m_detectionBrightnessMetricCache;
    m_detectionReliabilityMetricCache = other.m_detectionReliabilityMetricCache;
    m_detectionBrightnessRankCache = other.m_detectionBrightnessRankCache;
    m_detectionBrightnessRankIndices.clear();
}

QVector<CameraPlateSolver::SolverContext::CatalogStar> CameraPlateSolver::SolverContext::filterCatalogStars(const QVector<CatalogStar>& stars)
{
    QVector<CatalogStar> filtered;
    filtered.reserve(stars.size());
    QHash<QString, int> bestByName;

    for (const CatalogStar& star : stars)
    {
        if (!std::isfinite(star.rightAscensionDegrees)
            || !std::isfinite(star.declinationDegrees)
            || !std::isfinite(star.magnitude)
            || star.name.trimmed().isEmpty()
            || (star.magnitude > CameraSettings::m_maxPlateSolveMagnitude))
        {
            continue;
        }

        const QString normalizedName = star.name.trimmed().toUpper();
        const auto existingIt = bestByName.constFind(normalizedName);
        if (existingIt != bestByName.constEnd())
        {
            CatalogStar& existing = filtered[*existingIt];
            if (star.magnitude < existing.magnitude) {
                existing = star;
            }
            continue;
        }

        bestByName.insert(normalizedName, filtered.size());
        filtered.append(star);
    }

    std::sort(filtered.begin(), filtered.end(), [](const CatalogStar& lhs, const CatalogStar& rhs) {
        if (!qFuzzyCompare(lhs.magnitude + 1.0, rhs.magnitude + 1.0)) {
            return lhs.magnitude < rhs.magnitude;
        }
        return lhs.name < rhs.name;
    });

    return filtered;
}

QByteArray CameraPlateSolver::SolverContext::fetchSirilRangeFromSource(int chunkIndex, qint64 firstByte, qint64 lastByte, int sourceIndex)
{
    // Track 0a: in offline mode a cache miss is a hard, visible failure -- never a silent fetch.
    if (sirilOfflineModeEnabled())
    {
        qWarning().nospace() << "CameraPlateSolver: OFFLINE -- refusing Siril SPCC network fetch (cache miss)"
                   << " source=" << sirilSpccSourceName(sourceIndex)
                   << " chunk=" << chunkIndex << " bytes=" << firstByte << "-" << lastByte;
        return {};
    }

    if (!m_networkManager)
    {
        qWarning() << "CameraPlateSolver: Siril SPCC range request has no active network manager"
                   << "source" << sirilSpccSourceName(sourceIndex)
                   << "chunk" << chunkIndex << "bytes" << firstByte << lastByte;
        return {};
    }

    // Fast-exit if cancellation was already requested before we even start.
    if (isCancellationRequested()) {
        return {};
    }

    QNetworkRequest request(QUrl(sirilSpccChunkUrl(chunkIndex, sourceIndex)));
    request.setRawHeader("Range", QByteArray("bytes=%1-%2").replace("%1", QByteArray::number(firstByte)).replace("%2", QByteArray::number(lastByte)));
    request.setRawHeader("Accept-Encoding", "identity");
    request.setRawHeader("User-Agent", "SDRangel CameraPlateSolver/1.0");
    request.setAttribute(QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::NoLessSafeRedirectPolicy);
    request.setAttribute(QNetworkRequest::Http2AllowedAttribute, false);

    QNetworkReply *reply = m_networkManager->get(request);

    // Register the active reply so requestCancellation() can abort it while loop.exec() is
    // running. requestCancellation() can run on the feature thread (synchronous call chain from
    // Camera::applySettings), so guard the raw pointer with the owner's mutex.
    if (m_owner) {
        QMutexLocker activeReplyLocker(&m_owner->m_activeNetworkReplyMutex);
        m_owner->m_activeNetworkReply = reply;
    }

    QEventLoop loop;
    QTimer timeoutTimer;
    timeoutTimer.setSingleShot(true);
    QObject::connect(reply, &QNetworkReply::finished, &loop, &QEventLoop::quit);
    QObject::connect(&timeoutTimer, &QTimer::timeout, &loop, &QEventLoop::quit);
    timeoutTimer.start(30000);
    loop.exec();
    // A single-shot timer auto-deactivates when it fires, so capture whether it fired BEFORE we
    // stop it -- otherwise the stop() makes isActive() always false and the timedOut test below
    // degenerates to !reply->isFinished().
    const bool timerFired = !timeoutTimer.isActive();
    timeoutTimer.stop();  // stop the timer if the reply finished before it fired

    if (m_owner) {
        QMutexLocker activeReplyLocker(&m_owner->m_activeNetworkReplyMutex);
        m_owner->m_activeNetworkReply = nullptr;
    }

    QByteArray data;
    const bool timedOut = timerFired && !reply->isFinished();
    if (timedOut)
    {
        reply->abort();
        qWarning() << "CameraPlateSolver: Siril SPCC range request timed out"
                   << "source" << sirilSpccSourceName(sourceIndex)
                   << "chunk" << chunkIndex << "bytes" << firstByte << lastByte;
    }
    else if (reply->error() != QNetworkReply::NoError)
    {
        // Includes the OperationCanceledError case from requestNetworkCancellation().
        if (isCancellationRequested()) {
            qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: Siril SPCC range request cancelled";
        } else {
            qWarning() << "CameraPlateSolver: Siril SPCC range request failed"
                       << "source" << sirilSpccSourceName(sourceIndex)
                       << "chunk" << chunkIndex << "bytes" << firstByte << lastByte
                       << reply->errorString();
        }
    }
    else
    {
        const int httpStatus = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
        data = reply->readAll();
        // Require HTTP 206 Partial Content. A 200 OK full-file response cannot be
        // safely accepted here because fetchSirilRange() slices the returned bytes
        // using offset = firstByte - requestFirstByte (relative to the start of the
        // requested range), which is only valid when data[0] == byte requestFirstByte
        // of the file.  A 200 response starts at byte 0, so the slice would cut the
        // wrong bytes and cache corrupted catalog data.
        if ((httpStatus != 206) || data.isEmpty())
        {
            qWarning() << "CameraPlateSolver: Siril SPCC range request returned unexpected response"
                       << "source" << sirilSpccSourceName(sourceIndex)
                       << "chunk" << chunkIndex << "status" << httpStatus
                       << "bytes" << firstByte << lastByte << "size" << data.size()
                       << "contentRange" << reply->rawHeader("Content-Range");
            data.clear();
        }
    }
    reply->deleteLater();

    return data;
}

QByteArray CameraPlateSolver::SolverContext::fetchSirilRange(int chunkIndex, qint64 firstByte, qint64 lastByte)
{
    if ((chunkIndex < 0) || (firstByte < 0) || (lastByte < firstByte)) {
        return {};
    }

    const QString cacheKey = sirilRangeCacheKey(chunkIndex, firstByte, lastByte);
    const auto cachedRange = m_sirilRangeCache.constFind(cacheKey);
    if (cachedRange != m_sirilRangeCache.constEnd()) {
        return cachedRange.value();
    }
    const qint64 requestedSize = lastByte - firstByte + 1;
    const QByteArray cachedDiskRange = readSirilDiskCacheFile(
        sirilRangeDiskCachePath(chunkIndex, firstByte, lastByte),
        requestedSize);
    if (!cachedDiskRange.isEmpty())
    {
        m_sirilRangeCache.insert(cacheKey, cachedDiskRange);
        return cachedDiskRange;
    }

    const bool smallRange = requestedSize < kSirilMinRangeRequestSize;
    const qint64 requestFirstByte = smallRange
        ? (firstByte / kSirilMinRangeRequestSize) * kSirilMinRangeRequestSize
        : firstByte;
    const qint64 requestLastByte = smallRange
        ? std::max(lastByte, requestFirstByte + kSirilMinRangeRequestSize - 1)
        : lastByte;
    const QString requestCacheKey = sirilRangeCacheKey(chunkIndex, requestFirstByte, requestLastByte);
    if (requestCacheKey != cacheKey)
    {
        const auto it = m_sirilRangeCache.constFind(requestCacheKey);
        if (it != m_sirilRangeCache.constEnd())
        {
            const qint64 offset = firstByte - requestFirstByte;
            if (it.value().size() >= (offset + requestedSize)) {
                return it.value().mid(static_cast<int>(offset), static_cast<int>(requestedSize));
            }
        }
        const qint64 requestSize = requestLastByte - requestFirstByte + 1;
        const QByteArray cachedDiskRequest = readSirilDiskCacheFile(
            sirilRangeDiskCachePath(chunkIndex, requestFirstByte, requestLastByte),
            requestSize);
        if (!cachedDiskRequest.isEmpty())
        {
            const qint64 offset = firstByte - requestFirstByte;
            if (cachedDiskRequest.size() >= (offset + requestedSize))
            {
                const QByteArray requestedData = cachedDiskRequest.mid(static_cast<int>(offset), static_cast<int>(requestedSize));
                m_sirilRangeCache.insert(requestCacheKey, cachedDiskRequest);
                m_sirilRangeCache.insert(cacheKey, requestedData);
                writeSirilDiskCacheFile(sirilRangeDiskCachePath(chunkIndex, firstByte, lastByte), requestedData);
                return requestedData;
        }
    }
    }

    QByteArray data;
    for (int sourceIndex = 0; sourceIndex < 2 && data.isEmpty(); ++sourceIndex) {
        data = fetchSirilRangeFromSource(chunkIndex, requestFirstByte, requestLastByte, sourceIndex);
    }

    if (!data.isEmpty())
    {
        const qint64 offset = firstByte - requestFirstByte;
        if (data.size() < (offset + requestedSize))
        {
            qWarning() << "CameraPlateSolver: Siril SPCC expanded range request was too short"
                       << "chunk" << chunkIndex
                       << "requestedBytes" << firstByte << lastByte
                       << "fetchedBytes" << requestFirstByte << requestLastByte
                       << "size" << data.size();
            return {};
        }

        const QByteArray requestedData = data.mid(static_cast<int>(offset), static_cast<int>(requestedSize));
        m_sirilRangeCache.insert(requestCacheKey, data);
        m_sirilRangeCache.insert(cacheKey, requestedData);
        writeSirilDiskCacheFile(sirilRangeDiskCachePath(chunkIndex, requestFirstByte, requestLastByte), data);
        if (requestCacheKey != cacheKey) {
            writeSirilDiskCacheFile(sirilRangeDiskCachePath(chunkIndex, firstByte, lastByte), requestedData);
        }
        return requestedData;
    }

    return {};
}

QByteArray CameraPlateSolver::SolverContext::fetchSirilChunkIndex(int chunkIndex)
{
    const auto cachedIndex = m_sirilIndexCache.constFind(chunkIndex);
    if (cachedIndex != m_sirilIndexCache.constEnd()) {
        return cachedIndex.value();
    }

    const QByteArray cachedDiskIndex = readSirilDiskCacheFile(sirilIndexDiskCachePath(chunkIndex), kSirilIndexSize);
    if (!cachedDiskIndex.isEmpty())
    {
        m_sirilIndexCache.insert(chunkIndex, cachedDiskIndex);
        return cachedDiskIndex;
    }

    const qint64 firstByte = kSirilHeaderSize;
    const qint64 lastByte = kSirilHeaderSize + kSirilIndexSize - 1;
    const QByteArray indexBytes = fetchSirilRange(chunkIndex, firstByte, lastByte);
    if (indexBytes.size() != kSirilIndexSize)
    {
        qWarning() << "CameraPlateSolver: Siril SPCC chunk index request failed"
                   << "chunk" << chunkIndex
                   << "expected" << kSirilIndexSize
                   << "got" << indexBytes.size();
        // Do NOT negative-cache the failure: m_sirilIndexCache is never evicted and is persisted
        // across solves, so caching an empty result here would permanently blind this sky chunk
        // after a single transient network error. The chunk set is small/fixed, so re-requesting a
        // genuinely-missing chunk on the next solve is cheap.
        return {};
    }

    m_sirilIndexCache.insert(chunkIndex, indexBytes);
    writeSirilDiskCacheFile(sirilIndexDiskCachePath(chunkIndex), indexBytes);
    return indexBytes;
}

void CameraPlateSolver::SolverContext::prefetchSirilMergedRanges(const QVector<SirilMergedRange>& mergedRanges)
{
    if (!m_networkManager || mergedRanges.isEmpty()) {
        return;
    }

    struct PendingRange
    {
        SirilMergedRange range;
        QNetworkReply *reply = nullptr;
        QTimer *timer = nullptr;
    };

    QVector<SirilMergedRange> missingRanges;
    missingRanges.reserve(mergedRanges.size());
    for (const SirilMergedRange& range : mergedRanges)
    {
        const qint64 expectedByteCount = range.lastByte - range.firstByte + 1;
        const QString cacheKey = sirilRangeCacheKey(range.chunkIndex, range.firstByte, range.lastByte);
        if (m_sirilRangeCache.contains(cacheKey)) {
            continue;
        }

        const QByteArray cached = readSirilDiskCacheFile(
            sirilRangeDiskCachePath(range.chunkIndex, range.firstByte, range.lastByte),
            expectedByteCount);
        if (!cached.isEmpty())
        {
            m_sirilRangeCache.insert(cacheKey, cached);
            continue;
        }

        missingRanges.append(range);
    }

    if (missingRanges.isEmpty()) {
        return;
    }

    // Track 0a: in offline mode do not open any network requests for the missing ranges. Each is
    // reported so an incomplete snapshot is obvious; the downstream per-range fetch refuses too
    // (fetchSirilRangeFromSource) and the catalog load fails loudly rather than silently fetching.
    if (sirilOfflineModeEnabled())
    {
        const SirilMergedRange& first = missingRanges.first();
        qWarning().nospace() << "CameraPlateSolver: OFFLINE -- " << missingRanges.size()
                   << " Siril SPCC range(s) missing from cache; not fetching. First: chunk="
                   << first.chunkIndex << " bytes=" << first.firstByte << "-" << first.lastByte;
        return;
    }

    constexpr int kMaxConcurrentSirilRequests = 6;
    int nextRange = 0;
    int activeCount = 0;
    QEventLoop loop;
    QVector<PendingRange *> pending;

    const auto finishPending = [&](PendingRange *item) {
        if (!item || !item->reply) {
            return;
        }

        const SirilMergedRange range = item->range;
        const qint64 expectedByteCount = range.lastByte - range.firstByte + 1;
        QByteArray data;
        if ((item->reply->error() == QNetworkReply::NoError)
            && (item->reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt() == 206))
        {
            data = item->reply->readAll();
        }

        if (data.size() == expectedByteCount)
        {
            const QString cacheKey = sirilRangeCacheKey(range.chunkIndex, range.firstByte, range.lastByte);
            m_sirilRangeCache.insert(cacheKey, data);
            writeSirilDiskCacheFile(sirilRangeDiskCachePath(range.chunkIndex, range.firstByte, range.lastByte), data);
        }

        item->reply->deleteLater();
        item->reply = nullptr;
        if (item->timer)
        {
            item->timer->deleteLater();
            item->timer = nullptr;
        }
        --activeCount;
        if ((nextRange >= missingRanges.size()) && (activeCount <= 0)) {
            loop.quit();
        }
    };

    std::function<void()> startMore = [&]() {
        while ((activeCount < kMaxConcurrentSirilRequests)
            && (nextRange < missingRanges.size())
            && !isCancellationRequested())
        {
            const SirilMergedRange range = missingRanges[nextRange++];
            QNetworkRequest request(QUrl(sirilSpccChunkUrl(range.chunkIndex, 0)));
            request.setRawHeader("Range", QByteArray("bytes=%1-%2")
                .replace("%1", QByteArray::number(range.firstByte))
                .replace("%2", QByteArray::number(range.lastByte)));
            request.setRawHeader("Accept-Encoding", "identity");
            request.setRawHeader("User-Agent", "SDRangel CameraPlateSolver/1.0");
            request.setAttribute(QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::NoLessSafeRedirectPolicy);
            request.setAttribute(QNetworkRequest::Http2AllowedAttribute, false);

            PendingRange *item = new PendingRange{range, m_networkManager->get(request), new QTimer(&loop)};
            item->timer->setSingleShot(true);
            pending.append(item);
            ++activeCount;
            QObject::connect(item->reply, &QNetworkReply::finished, &loop, [&, item]() {
                finishPending(item);
                startMore();
            });
            QObject::connect(item->timer, &QTimer::timeout, &loop, [item]() {
                if (item->reply) {
                    item->reply->abort();
                }
            });
            item->timer->start(30000);
        }

        if ((nextRange >= missingRanges.size()) && (activeCount <= 0)) {
            loop.quit();
        }
    };

    // Quit promptly if plate solving is cancelled mid-prefetch; the cleanup below aborts the
    // in-flight replies. Without this the loop would wait for the active replies to finish or hit
    // their 30s timeout (requestCancellation only aborts the single-fetch reply, not these).
    QTimer cancelCheckTimer;
    QObject::connect(&cancelCheckTimer, &QTimer::timeout, &loop, [&]() {
        if (isCancellationRequested()) {
            loop.quit();
        }
    });
    cancelCheckTimer.start(100);
    startMore();
    if (activeCount > 0) {
        loop.exec();
    }

    for (PendingRange *item : pending)
    {
        if (item && item->reply)
        {
            // Disconnect from the loop first: abort() can emit finished() synchronously, which would
            // run finishPending() and set item->reply = nullptr, turning the deleteLater() below into
            // a null-pointer dereference.
            QObject::disconnect(item->reply, nullptr, &loop, nullptr);
            item->reply->abort();
            item->reply->deleteLater();
        }
        delete item;
    }
}

CameraPlateSolver::SolverContext::SirilQueryGeometry CameraPlateSolver::SolverContext::sirilQueryGeometry(const CameraSettings& settings, const QSize& imageSize, const QDateTime& captureDateTimeUtc)
{
    SirilQueryGeometry geometry;
    if (!plateSolveStartUsesDirection(settings))
    {
        geometry.failureReason = QStringLiteral("Siril SPCC Gaia DR3 unavailable for start mode");
        return geometry;
    }

    const RADec centerRaDec = Astronomy::azAltToRaDec(
        AzAlt{settings.m_azimuth, settings.m_elevation},
        settings.m_latitude,
        settings.m_longitude,
        captureDateTimeUtc);
    geometry.centerRaDegrees = normalizeDegrees(centerRaDec.ra * 15.0);
    geometry.centerDecDegrees = centerRaDec.dec;
    if (!std::isfinite(geometry.centerRaDegrees) || !std::isfinite(geometry.centerDecDegrees))
    {
        geometry.failureReason = QStringLiteral("Siril SPCC Gaia DR3 unavailable for invalid pointing");
        return geometry;
    }

    const double diagonalFov = fovLongEdgeDiagonalDegrees(imageSize, settings.m_fov);
    geometry.queryRadiusDegrees = std::max(0.5, diagonalFov * 0.5 + settings.m_plateSolveAzElSearchRadius + 1.0);
    if (geometry.queryRadiusDegrees > kSirilMaxQueryRadiusDegrees)
    {
        geometry.tooWide = true;
        geometry.failureReason = QStringLiteral("Siril SPCC Gaia DR3 unavailable for wide query");
        return geometry;
    }

    geometry.valid = true;
    return geometry;
}

bool CameraPlateSolver::SolverContext::sirilCellRecordRange(quint32 pixel, int& chunkIndex, qint64& firstRecord, qint64& recordCount)
{
    chunkIndex = static_cast<int>(pixel / kSirilPixelsPerChunk);
    const int localPixel = static_cast<int>(pixel % kSirilPixelsPerChunk);
    const QByteArray indexBytes = fetchSirilChunkIndex(chunkIndex);
    if (indexBytes.size() != kSirilIndexSize) {
        return false;
    }

    const char *cellEndBytes = indexBytes.constData() + static_cast<qint64>(localPixel) * sizeof(quint32);
    const quint32 cellStart = (localPixel > 0)
        ? qFromLittleEndian<quint32>(reinterpret_cast<const uchar *>(cellEndBytes - sizeof(quint32)))
        : 0;
    const quint32 cellEnd = qFromLittleEndian<quint32>(reinterpret_cast<const uchar *>(cellEndBytes));
    const quint32 chunkRecordCount = qFromLittleEndian<quint32>(
        reinterpret_cast<const uchar *>(indexBytes.constData() + kSirilIndexSize - sizeof(quint32)));
    if ((cellEnd < cellStart) || (cellEnd > chunkRecordCount))
    {
        qWarning() << "CameraPlateSolver: Siril SPCC invalid cell index"
                   << "chunk" << chunkIndex
                   << "localPixel" << localPixel
                   << "cellStart" << cellStart
                   << "cellEnd" << cellEnd
                   << "chunkRecords" << chunkRecordCount;
        return false;
    }

    firstRecord = cellStart;
    recordCount = static_cast<qint64>(cellEnd) - static_cast<qint64>(cellStart);
    if (recordCount > kSirilMaxRecordsPerCell)
    {
        qWarning() << "CameraPlateSolver: Siril SPCC cell has too many records"
                   << "chunk" << chunkIndex
                   << "localPixel" << localPixel
                   << "records" << recordCount
                   << "max" << kSirilMaxRecordsPerCell;
        return false;
    }

    return true;
}

QVector<CameraPlateSolver::SolverContext::CatalogStar> CameraPlateSolver::SolverContext::loadSirilAstroCatalog(const CameraSettings& settings, const QSize& imageSize, const QDateTime& captureDateTimeUtc, double maxMagnitude, QString* catalogSource)
{
    QVector<CatalogStar> stars;
    const SirilQueryGeometry geometry = sirilQueryGeometry(settings, imageSize, captureDateTimeUtc);
    if (!geometry.valid)
    {
        if (catalogSource) {
            *catalogSource = geometry.failureReason;
        }
        if (geometry.tooWide)
        {
            qWarning() << "CameraPlateSolver: Siril Gaia astrometric query is too wide, falling back to HYG/bundled"
                       << "radius" << geometry.queryRadiusDegrees
                       << "max" << kSirilMaxQueryRadiusDegrees;
        }
        return stars;
    }

    const QString catalogPath = sirilAstroCatalogPath();
    if (!QFileInfo::exists(catalogPath))
    {
        if (QFileInfo::exists(sirilAstroCompressedCatalogPath()))
        {
            qWarning() << "CameraPlateSolver: Siril Gaia astrometric catalog is compressed; decompress before use"
                       << "compressed" << sirilAstroCompressedCatalogPath()
                       << "expected" << catalogPath;
        }
        else
        {
            qWarning() << "CameraPlateSolver: Siril Gaia astrometric catalog is not installed"
                       << "expected" << catalogPath;
        }
        if (catalogSource) {
            *catalogSource = QStringLiteral("Siril Gaia DR3 Astrometric unavailable");
        }
        return stars;
    }

    const double centerRaDegrees = geometry.centerRaDegrees;
    const double centerDecDegrees = geometry.centerDecDegrees;
    const double queryRadius = geometry.queryRadiusDegrees;
    const QString regionCachePath = sirilAstroRegionDiskCachePath(
        centerRaDegrees,
        centerDecDegrees,
        queryRadius,
        maxMagnitude);
    stars = readSirilRegionDiskCacheFile(regionCachePath);
    if (!stars.isEmpty())
    {
        if (catalogSource) {
            *catalogSource = QStringLiteral("Siril Gaia DR3 Astrometric");
        }
        qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: loaded cached Siril Gaia astrometric stars"
                 << stars.size()
                 << "center RA" << centerRaDegrees
                 << "Dec" << centerDecDegrees
                 << "radius" << queryRadius
                 << "maxMag" << maxMagnitude;
        return stars;
    }

    QFile file(catalogPath);
    if (!file.open(QIODevice::ReadOnly))
    {
        qWarning() << "CameraPlateSolver: Siril Gaia astrometric catalog open failed"
                   << "path" << catalogPath
                   << "error" << file.errorString();
        if (catalogSource) {
            *catalogSource = QStringLiteral("Siril Gaia DR3 Astrometric unavailable");
        }
        return stars;
    }

    const QByteArray indexBytes = readSirilAstroIndex(file);
    if (indexBytes.isEmpty())
    {
        if (catalogSource) {
            *catalogSource = QStringLiteral("Siril Gaia DR3 Astrometric unavailable");
        }
        return stars;
    }

    const QSet<quint32> pixels = sampleSirilHealpixPixels(centerRaDegrees, centerDecDegrees, queryRadius);
    QVector<SirilCellRange> cellRanges;
    cellRanges.reserve(pixels.size());
    for (quint32 pixel : pixels)
    {
        qint64 firstRecord = 0;
        qint64 recordCount = 0;
        if (!sirilAstroCellRecordRange(indexBytes, pixel, firstRecord, recordCount) || (recordCount <= 0)) {
            continue;
        }

        const qint64 dataStart = kSirilHeaderSize
            + static_cast<qint64>(kSirilAstroIndexSize)
            + firstRecord * kSirilAstroRecordSize;
        const qint64 recordByteCount = recordCount * kSirilAstroRecordSize;
        cellRanges.append({
            0,
            firstRecord,
            recordCount,
            dataStart,
            dataStart + recordByteCount - 1
        });
    }

    std::sort(cellRanges.begin(), cellRanges.end(), [](const SirilCellRange& lhs, const SirilCellRange& rhs) {
        return lhs.firstByte < rhs.firstByte;
    });

    QVector<SirilMergedRange> mergedRanges;
    for (int cellIndex = 0; cellIndex < cellRanges.size(); ++cellIndex)
    {
        const SirilCellRange& cell = cellRanges[cellIndex];
        if (!mergedRanges.isEmpty())
        {
            SirilMergedRange& range = mergedRanges.last();
            const qint64 mergedLastByte = std::max(range.lastByte, cell.lastByte);
            const bool canMerge = (cell.firstByte <= (range.lastByte + kSirilMaxMergedRangeGapBytes + 1))
                && ((mergedLastByte - range.firstByte + 1) <= kSirilMaxMergedRangeRequestSize);
            if (canMerge)
            {
                range.lastByte = mergedLastByte;
                range.cellIndexes.append(cellIndex);
                continue;
            }
        }

        mergedRanges.append({
            0,
            cell.firstByte,
            cell.lastByte,
            QVector<int>{cellIndex}
        });
    }

    QSet<quint64> seenStars;
    stars.reserve(cellRanges.size() * 16);
    qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: Siril Gaia astrometric request"
             << "pixels" << pixels.size()
             << "cells" << cellRanges.size()
             << "ranges" << mergedRanges.size()
             << "center RA" << centerRaDegrees
             << "Dec" << centerDecDegrees
             << "radius" << queryRadius
             << "maxMag" << maxMagnitude;

    for (const SirilMergedRange& range : mergedRanges)
    {
        if (isCancellationRequested()) {
            break;
        }
        const qint64 expectedByteCount = range.lastByte - range.firstByte + 1;
        if ((expectedByteCount <= 0) || (expectedByteCount > std::numeric_limits<int>::max())) {
            continue;
        }
        if (!file.seek(range.firstByte))
        {
            qWarning() << "CameraPlateSolver: Siril Gaia astrometric seek failed"
                       << "offset" << range.firstByte
                       << "error" << file.errorString();
            continue;
        }

        const QByteArray recordBytes = file.read(expectedByteCount);
        if (recordBytes.size() != expectedByteCount)
        {
            qWarning() << "CameraPlateSolver: Siril Gaia astrometric record read failed"
                       << "expectedBytes" << expectedByteCount
                       << "got" << recordBytes.size();
            continue;
        }

        for (int cellIndex : range.cellIndexes)
        {
            const SirilCellRange& cell = cellRanges[cellIndex];
            const qint64 cellOffset = cell.firstByte - range.firstByte;
            for (qint64 recordIndex = 0; recordIndex < cell.recordCount; ++recordIndex)
            {
                const qint64 recordOffset = cellOffset + recordIndex * kSirilAstroRecordSize;
                if ((recordOffset < 0) || ((recordOffset + kSirilAstroRecordSize) > recordBytes.size())) {
                    break;
                }

                const char *record = recordBytes.constData() + recordOffset;
                const qint32 rawRa = qFromLittleEndian<qint32>(reinterpret_cast<const uchar *>(record));
                const qint32 rawDec = qFromLittleEndian<qint32>(reinterpret_cast<const uchar *>(record + 4));
                const qint16 rawMag = qFromLittleEndian<qint16>(reinterpret_cast<const uchar *>(record + 14));
                const double starRaDegrees = normalizeDegrees(static_cast<double>(rawRa) * kSirilAngleScale);
                const double starDecDegrees = static_cast<double>(rawDec) * kSirilAngleScale;
                const double magnitude = static_cast<double>(rawMag) / 1000.0;
                if (!std::isfinite(starRaDegrees)
                    || !std::isfinite(starDecDegrees)
                    || !std::isfinite(magnitude)
                    || (magnitude > maxMagnitude)
                    || (angularSeparationDegrees(centerRaDegrees, centerDecDegrees, starRaDegrees, starDecDegrees) > queryRadius))
                {
                    continue;
                }

                const quint64 starKey = (static_cast<quint64>(static_cast<quint32>(rawRa)) << 32)
                    | static_cast<quint32>(rawDec);
                if (seenStars.contains(starKey)) {
                    continue;
                }
                seenStars.insert(starKey);
                stars.append({
                    formatGaiaCoordinateLabel(starRaDegrees, starDecDegrees),
                    starRaDegrees,
                    starDecDegrees,
                    magnitude,
                    QString()
                });
            }
        }
    }

    std::sort(stars.begin(), stars.end(), [](const CatalogStar& lhs, const CatalogStar& rhs) {
        return lhs.magnitude < rhs.magnitude;
    });

    if (catalogSource) {
        *catalogSource = QStringLiteral("Siril Gaia DR3 Astrometric");
    }
    writeSirilRegionDiskCacheFile(regionCachePath, stars, settings.m_starCatalogDiskCacheSizeGb);
    qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: loaded Siril Gaia astrometric stars"
             << stars.size()
             << "pixels" << pixels.size()
             << "center RA" << centerRaDegrees
             << "Dec" << centerDecDegrees
             << "radius" << queryRadius
             << "maxMag" << maxMagnitude;
    return stars;
}

QVector<CameraPlateSolver::SolverContext::CatalogStar> CameraPlateSolver::SolverContext::loadSirilSpccCatalog(const CameraSettings& settings, const QSize& imageSize, const QDateTime& captureDateTimeUtc, double maxMagnitude, QString* catalogSource)
{
    QVector<CatalogStar> stars;
    const SirilQueryGeometry geometry = sirilQueryGeometry(settings, imageSize, captureDateTimeUtc);
    if (!geometry.valid)
    {
        if (catalogSource) {
            *catalogSource = geometry.failureReason;
        }
        if (geometry.tooWide)
        {
            qWarning() << "CameraPlateSolver: Siril SPCC Gaia query is too wide, falling back to HYG/bundled"
                       << "radius" << geometry.queryRadiusDegrees
                       << "max" << kSirilMaxQueryRadiusDegrees;
        }
        return stars;
    }
    const double centerRaDegrees = geometry.centerRaDegrees;
    const double centerDecDegrees = geometry.centerDecDegrees;
    const double queryRadius = geometry.queryRadiusDegrees;
    const QString regionCachePath = sirilRegionDiskCachePath(
        centerRaDegrees,
        centerDecDegrees,
        queryRadius,
        maxMagnitude);
    stars = readSirilRegionDiskCacheFile(regionCachePath);
    if (!stars.isEmpty())
    {
        if (catalogSource) {
            *catalogSource = QStringLiteral("Siril SPCC Gaia DR3");
        }
        qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: loaded cached Siril SPCC Gaia stars"
                 << stars.size()
                 << "center RA" << centerRaDegrees
                 << "Dec" << centerDecDegrees
                 << "radius" << queryRadius
                 << "maxMag" << maxMagnitude;
        return stars;
    }

    const QSet<quint32> pixels = sampleSirilHealpixPixels(centerRaDegrees, centerDecDegrees, queryRadius);
    QVector<SirilCellRange> cellRanges;
    cellRanges.reserve(pixels.size());
    for (quint32 pixel : pixels)
    {
        int chunkIndex = 0;
        qint64 firstRecord = 0;
        qint64 recordCount = 0;
        if (!sirilCellRecordRange(pixel, chunkIndex, firstRecord, recordCount) || (recordCount <= 0)) {
            continue;
        }

        const qint64 dataStart = kSirilHeaderSize
            + static_cast<qint64>(kSirilPixelsPerChunk) * sizeof(quint32)
            + firstRecord * kSirilRecordSize;
        const qint64 recordByteCount = recordCount * kSirilRecordSize;
        if (recordByteCount > std::numeric_limits<int>::max())
        {
            qWarning() << "CameraPlateSolver: Siril SPCC record range is too large"
                       << "chunk" << chunkIndex
                       << "firstRecord" << firstRecord
                       << "records" << recordCount
                       << "bytes" << recordByteCount;
            continue;
        }

        cellRanges.append({
            chunkIndex,
            firstRecord,
            recordCount,
            dataStart,
            dataStart + recordByteCount - 1
        });
    }

    std::sort(cellRanges.begin(), cellRanges.end(), [](const SirilCellRange& lhs, const SirilCellRange& rhs) {
        if (lhs.chunkIndex != rhs.chunkIndex) {
            return lhs.chunkIndex < rhs.chunkIndex;
        }
        return lhs.firstByte < rhs.firstByte;
    });

    QVector<SirilMergedRange> mergedRanges;
    for (int cellIndex = 0; cellIndex < cellRanges.size(); ++cellIndex)
    {
        const SirilCellRange& cell = cellRanges[cellIndex];
        if (!mergedRanges.isEmpty())
        {
            SirilMergedRange& range = mergedRanges.last();
            const qint64 mergedLastByte = std::max(range.lastByte, cell.lastByte);
            const bool canMerge = (range.chunkIndex == cell.chunkIndex)
                && (cell.firstByte <= (range.lastByte + kSirilMaxMergedRangeGapBytes + 1))
                && ((mergedLastByte - range.firstByte + 1) <= kSirilMaxMergedRangeRequestSize);
            if (canMerge)
            {
                range.lastByte = mergedLastByte;
                range.cellIndexes.append(cellIndex);
                continue;
            }
        }

        mergedRanges.append({
            cell.chunkIndex,
            cell.firstByte,
            cell.lastByte,
            QVector<int>{cellIndex}
        });
    }

    QSet<quint64> seenStars;
    stars.reserve(cellRanges.size() * 8);

    qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: Siril SPCC Gaia request"
             << "pixels" << pixels.size()
             << "cells" << cellRanges.size()
             << "ranges" << mergedRanges.size()
             << "center RA" << centerRaDegrees
             << "Dec" << centerDecDegrees
             << "radius" << queryRadius
             << "maxMag" << maxMagnitude;

    prefetchSirilMergedRanges(mergedRanges);

    for (const SirilMergedRange& range : mergedRanges)
    {
        if (isCancellationRequested()) {
            break;
        }
        const qint64 expectedByteCount = range.lastByte - range.firstByte + 1;
        if (expectedByteCount > std::numeric_limits<int>::max())
        {
            qWarning() << "CameraPlateSolver: Siril SPCC merged record range is too large"
                       << "chunk" << range.chunkIndex
                       << "bytes" << expectedByteCount;
            continue;
        }

        const QByteArray recordBytes = fetchSirilRange(range.chunkIndex, range.firstByte, range.lastByte);
        if (recordBytes.size() != expectedByteCount)
        {
            qWarning() << "CameraPlateSolver: Siril SPCC merged record range request failed"
                       << "chunk" << range.chunkIndex
                       << "cells" << range.cellIndexes.size()
                       << "expectedBytes" << expectedByteCount
                       << "got" << recordBytes.size();
            continue;
        }

        for (int cellIndex : range.cellIndexes)
        {
            const SirilCellRange& cell = cellRanges[cellIndex];
            const qint64 cellOffset = cell.firstByte - range.firstByte;
            for (qint64 recordIndex = 0; recordIndex < cell.recordCount; ++recordIndex)
            {
                const qint64 recordOffset = cellOffset + recordIndex * kSirilRecordSize;
                if ((recordOffset < 0) || ((recordOffset + kSirilRecordSize) > recordBytes.size()))
                {
                    qWarning() << "CameraPlateSolver: Siril SPCC record offset outside merged range"
                               << "chunk" << range.chunkIndex
                               << "firstRecord" << cell.firstRecord
                               << "records" << cell.recordCount
                               << "offset" << recordOffset
                               << "size" << recordBytes.size();
                    break;
                }

                const char *record = recordBytes.constData() + recordOffset;
                const qint32 rawRa = qFromLittleEndian<qint32>(reinterpret_cast<const uchar *>(record));
                const qint32 rawDec = qFromLittleEndian<qint32>(reinterpret_cast<const uchar *>(record + 4));
                const qint16 rawMag = qFromLittleEndian<qint16>(reinterpret_cast<const uchar *>(record + 12));
                const double starRaDegrees = normalizeDegrees(static_cast<double>(rawRa) * kSirilAngleScale);
                const double starDecDegrees = static_cast<double>(rawDec) * kSirilAngleScale;
                const double magnitude = static_cast<double>(rawMag) / 1000.0;
                if (!std::isfinite(starRaDegrees)
                    || !std::isfinite(starDecDegrees)
                    || !std::isfinite(magnitude)
                    || (magnitude > maxMagnitude)
                    || (angularSeparationDegrees(centerRaDegrees, centerDecDegrees, starRaDegrees, starDecDegrees) > queryRadius))
                {
                    continue;
                }

                const quint64 starKey = (static_cast<quint64>(static_cast<quint32>(rawRa)) << 32)
                    | static_cast<quint32>(rawDec);
                if (seenStars.contains(starKey)) {
                    continue;
                }
                seenStars.insert(starKey);
                stars.append({
                    formatGaiaCoordinateLabel(starRaDegrees, starDecDegrees),
                    starRaDegrees,
                    starDecDegrees,
                    magnitude,
                    QString()
                });
            }
        }
    }

    std::sort(stars.begin(), stars.end(), [](const CatalogStar& lhs, const CatalogStar& rhs) {
        return lhs.magnitude < rhs.magnitude;
    });

    if (catalogSource) {
        *catalogSource = QStringLiteral("Siril SPCC Gaia DR3");
    }
    writeSirilRegionDiskCacheFile(regionCachePath, stars, settings.m_starCatalogDiskCacheSizeGb);
    // Bound the raw SPCC byte-range disk cache (siril-spcc-cache/v1/ranges/*.bin) with the same
    // limit as the region cache. This cache is written per byte-range fetch and had no eviction,
    // so it grew unbounded (observed at 148 GB, filling the disk). Enforced once here at the end of
    // a network catalog build (all of this solve's ranges are already fetched, so eviction only
    // drops older regions' files, never this solve's). The index sub-dir is naturally bounded
    // (<= 48 chunks) and holds .idx files the *.bin evictor ignores, so it is left alone.
    enforceSirilRegionDiskCacheLimit(
        QDir(sirilCacheRootDir()).filePath(QStringLiteral("ranges")),
        settings.m_starCatalogDiskCacheSizeGb);
    qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: loaded Siril SPCC Gaia stars"
             << stars.size()
             << "pixels" << pixels.size()
             << "center RA" << centerRaDegrees
             << "Dec" << centerDecDegrees
             << "radius" << queryRadius
             << "maxMag" << maxMagnitude;
    return stars;
}

void CameraPlateSolver::SolverContext::prepareDetectionMetricCache(const QVector<CameraPipelineStarDetection>& starDetections)
{
    m_detectionBrightnessMetricCache.resize(starDetections.size());
    m_detectionReliabilityMetricCache.resize(starDetections.size());
    m_detectionBrightnessRankCache.resize(starDetections.size());
    std::fill(m_detectionBrightnessRankCache.begin(), m_detectionBrightnessRankCache.end(), 0.5);
    m_detectionBrightnessRankIndices.clear();
    for (int i = 0; i < starDetections.size(); ++i)
    {
        m_detectionBrightnessMetricCache[i] = detectionBrightnessMetric(starDetections[i]);
        m_detectionReliabilityMetricCache[i] = detectionReliabilityMetric(starDetections[i]);
    }
}

QVector<int> CameraPlateSolver::SolverContext::selectDetectionIndicesForSolve(const QVector<CameraPipelineStarDetection>& starDetections, const QSize& imageSize)
{
    const int detectionCount = static_cast<int>(starDetections.size());
    QVector<int> byReliability;
    byReliability.reserve(detectionCount);
    for (int i = 0; i < detectionCount; ++i) {
        byReliability.append(i);
    }

    std::sort(byReliability.begin(), byReliability.end(), [this, &starDetections](int lhs, int rhs) {
        if (starDetections[lhs].m_hotPixelSuspect != starDetections[rhs].m_hotPixelSuspect) {
            return !starDetections[lhs].m_hotPixelSuspect;
        }
        const double lhsReliability = cachedDetectionReliabilityMetric(starDetections, lhs);
        const double rhsReliability = cachedDetectionReliabilityMetric(starDetections, rhs);
        if (!qFuzzyCompare(lhsReliability + 1.0, rhsReliability + 1.0)) {
            return lhsReliability > rhsReliability;
        }
        const double lhsBrightness = cachedDetectionBrightnessMetric(starDetections, lhs);
        const double rhsBrightness = cachedDetectionBrightnessMetric(starDetections, rhs);
        if (!qFuzzyCompare(lhsBrightness + 1.0, rhsBrightness + 1.0)) {
            return lhsBrightness > rhsBrightness;
        }
        if (starDetections[lhs].m_peakValue != starDetections[rhs].m_peakValue) {
            return starDetections[lhs].m_peakValue > starDetections[rhs].m_peakValue;
        }
        if (starDetections[lhs].m_saturated != starDetections[rhs].m_saturated) {
            return !starDetections[lhs].m_saturated;
        }
        return starDetections[lhs].m_roundness > starDetections[rhs].m_roundness;
    });

    QVector<int> byBrightness = byReliability;
    std::sort(byBrightness.begin(), byBrightness.end(), [this, &starDetections](int lhs, int rhs) {
        if (starDetections[lhs].m_hotPixelSuspect != starDetections[rhs].m_hotPixelSuspect) {
            return !starDetections[lhs].m_hotPixelSuspect;
        }
        const double lhsBrightness = cachedDetectionBrightnessMetric(starDetections, lhs);
        const double rhsBrightness = cachedDetectionBrightnessMetric(starDetections, rhs);
        if (!qFuzzyCompare(lhsBrightness + 1.0, rhsBrightness + 1.0)) {
            return lhsBrightness > rhsBrightness;
        }
        return cachedDetectionReliabilityMetric(starDetections, lhs)
            > cachedDetectionReliabilityMetric(starDetections, rhs);
    });

    QVector<int> ranked;
    ranked.reserve(std::min(detectionCount, kMaxDetectionsForSolve * 3));
    QSet<int> seen;
    seen.reserve(std::min(detectionCount, kMaxDetectionsForSolve * 3));
    const auto appendUnique = [&ranked, &seen](int index) {
        if (seen.contains(index)) {
            return;
        }
        seen.insert(index);
        ranked.append(index);
    };
    for (int i = 0; (i < detectionCount) && (ranked.size() < kMaxDetectionsForSolve * 3); ++i)
    {
        appendUnique(byReliability[i]);
        appendUnique(byBrightness[i]);
    }
    if (ranked.isEmpty()) {
        return ranked;
    }

    // Prefer stars spread across the image: reject candidates too close to already-selected ones.
    // This improves the geometric diversity of triangle/quad patterns used for blind matching.
    const double minSpreadPixels = std::min(imageSize.width(), imageSize.height()) / (kMaxDetectionsForSolve * 0.75);
    const double minSpreadSquared = minSpreadPixels * minSpreadPixels;
    QVector<int> spread;
    spread.reserve(std::min(static_cast<int>(ranked.size()), kMaxDetectionsForSolve));
    const auto appendSpreadCandidate = [&spread, &starDetections, minSpreadSquared](int candidate) {
        const QPointF& candidatePos = starDetections[candidate].m_center;
        for (int selected : spread)
        {
            const double dx = candidatePos.x() - starDetections[selected].m_center.x();
            const double dy = candidatePos.y() - starDetections[selected].m_center.y();
            if ((dx * dx + dy * dy) < minSpreadSquared) {
                return false;
            }
        }
        spread.append(candidate);
        return true;
    };

    QVector<int> broadSaturated;
    broadSaturated.reserve(detectionCount);
    for (int i = 0; i < detectionCount; ++i)
    {
        const CameraPipelineStarDetection& detection = starDetections[i];
        if (!detection.m_hotPixelSuspect
            && detection.m_saturated
            && (detection.m_fwhm >= 3.0f))
        {
            broadSaturated.append(i);
        }
    }
    std::sort(broadSaturated.begin(), broadSaturated.end(), [this, &starDetections](int lhs, int rhs) {
        const double lhsScore = cachedDetectionBrightnessMetric(starDetections, lhs)
            * std::sqrt(std::max(1.0f, starDetections[lhs].m_fwhm));
        const double rhsScore = cachedDetectionBrightnessMetric(starDetections, rhs)
            * std::sqrt(std::max(1.0f, starDetections[rhs].m_fwhm));
        if (!qFuzzyCompare(lhsScore + 1.0, rhsScore + 1.0)) {
            return lhsScore > rhsScore;
        }
        return cachedDetectionReliabilityMetric(starDetections, lhs)
            > cachedDetectionReliabilityMetric(starDetections, rhs);
    });
    const int broadSaturatedLimit = std::min(64, static_cast<int>(broadSaturated.size()));
    for (int i = 0; (i < broadSaturatedLimit) && (spread.size() < kMaxDetectionsForSolve); ++i) {
        appendSpreadCandidate(broadSaturated[i]);
    }

    for (int candidate : ranked) {
        if (spread.contains(candidate)) {
            continue;
        }
        appendSpreadCandidate(candidate);
        if (spread.size() >= kMaxDetectionsForSolve) {
            break;
        }
    }
    if (spread.size() >= 4) {
        return spread;
    }

    if (ranked.size() > kMaxDetectionsForSolve) {
        ranked.resize(kMaxDetectionsForSolve);
    }
    return ranked;
}

CameraPlateSolver::SolverContext::PlateSolveCatalogContext CameraPlateSolver::SolverContext::buildPlateSolveCatalogContext(const CameraSettings& settings, const QSize& imageSize, const QDateTime& captureDateTimeUtc, double maxMagnitude, double catalogLoadMaxMagnitude)
{
    // The outer solve retries (recenter ladder, bright-catalog, escapes) rebuild this
    // context for every run — each a multi-100k-star cache-file parse plus the alias and
    // bright-star merge passes (~0.5 s) — and frequently with byte-identical inputs (the
    // recenter ladder revisits offsets; bright/initial runs share the seed). Memoize on
    // the exact inputs: a hit returns the same context the build would have produced.
    // QVector's implicit sharing keeps cached copies cheap until a caller writes.
    struct CatalogContextCacheEntry
    {
        QString key;
        PlateSolveCatalogContext context;
    };
    static QMutex s_contextCacheMutex;
    static QVector<CatalogContextCacheEntry> s_contextCache;
    // Large regions run ~50 MB of catalog data each; keep the cache small — the wins
    // come from immediate repeats within one outer solve (same-seed runs, revisited
    // recenter offsets), not from long retention.
    constexpr int kMaxCachedContexts = 3;

    const QString cacheKey = QStringLiteral("%1|%2|%3|%4|%5|%6|%7|%8|%9|%10|%11")
        .arg(static_cast<int>(settings.m_plateSolveCatalogSource))
        .arg(qRound64(static_cast<double>(settings.m_azimuth) * 1e4))
        .arg(qRound64(static_cast<double>(settings.m_elevation) * 1e4))
        .arg(qRound64(static_cast<double>(settings.m_fov) * 1e4))
        .arg(qRound64(static_cast<double>(settings.m_plateSolveAzElSearchRadius) * 1e4))
        .arg(imageSize.width())
        .arg(imageSize.height())
        .arg(captureDateTimeUtc.toSecsSinceEpoch())
        .arg(qRound64(maxMagnitude * 1e3))
        .arg(qRound64(catalogLoadMaxMagnitude * 1e3))
        .arg(settings.m_plateSolveMinMatches)
        // Observer location: the az/el seed maps to the RA/Dec catalog region via
        // sirilQueryGeometry()/buildRaDecToAzAltParams(), which both use lat/lon, so a site
        // change with the same az/el/time/FoV must NOT reuse the previous location's context.
        + QStringLiteral("|%1|%2")
            .arg(qRound64(static_cast<double>(settings.m_latitude) * 1e5))
            .arg(qRound64(static_cast<double>(settings.m_longitude) * 1e5));
    {
        QMutexLocker locker(&s_contextCacheMutex);
        for (int i = 0; i < s_contextCache.size(); ++i)
        {
            if (s_contextCache[i].key == cacheKey)
            {
                // Refresh LRU position.
                CatalogContextCacheEntry entry = s_contextCache.takeAt(i);
                s_contextCache.append(entry);
                return entry.context;
            }
        }
    }

    PlateSolveCatalogContext context;
    const double effectiveCatalogLoadMaxMagnitude = (catalogLoadMaxMagnitude > 0.0)
        ? std::max(maxMagnitude, catalogLoadMaxMagnitude)
        : maxMagnitude;
    const bool autoSiril = (settings.m_plateSolveCatalogSource == CameraSettings::PlateSolveCatalogAuto)
        && plateSolveStartUsesDirection(settings)
        && (settings.m_fov <= kSirilAutoMaxFovDegrees);
    const bool requestSirilAstro = (settings.m_plateSolveCatalogSource == CameraSettings::PlateSolveCatalogSirilAstroGaia)
        || ((settings.m_plateSolveCatalogSource == CameraSettings::PlateSolveCatalogAuto)
            && autoSiril
            && QFileInfo::exists(sirilAstroCatalogPath()));
    const bool requestSirilSpcc = (settings.m_plateSolveCatalogSource == CameraSettings::PlateSolveCatalogSirilSpccGaia)
        || (autoSiril && !requestSirilAstro);
    if (requestSirilAstro)
    {
        context.catalogStars = loadSirilAstroCatalog(
            settings,
            imageSize,
            captureDateTimeUtc,
            effectiveCatalogLoadMaxMagnitude,
            &context.catalogSource);
        if (!context.catalogStars.isEmpty())
        {
            const SirilQueryGeometry geometry = sirilQueryGeometry(settings, imageSize, captureDateTimeUtc);
            applyNamedAliasesToCatalog(settings, context.catalogStars);
            mergeBundledBrightStarsIntoCatalog(
                settings,
                context.catalogStars,
                effectiveCatalogLoadMaxMagnitude,
                geometry.valid ? geometry.centerRaDegrees : std::numeric_limits<double>::quiet_NaN(),
                geometry.valid ? geometry.centerDecDegrees : std::numeric_limits<double>::quiet_NaN(),
                geometry.valid ? geometry.queryRadiusDegrees : -1.0);
        }
        if (context.catalogStars.size() < settings.m_plateSolveMinMatches)
        {
            qWarning() << "CameraPlateSolver: Siril Gaia astrometric catalog did not provide enough stars, falling back to HYG/bundled"
                       << "stars" << context.catalogStars.size()
                       << "minMatches" << settings.m_plateSolveMinMatches;
            context.catalogStars.clear();
            context.catalogSource.clear();
        }
    }

    if (context.catalogStars.isEmpty() && requestSirilSpcc)
    {
        context.catalogStars = loadSirilSpccCatalog(
            settings,
            imageSize,
            captureDateTimeUtc,
            effectiveCatalogLoadMaxMagnitude,
            &context.catalogSource);
        if (!context.catalogStars.isEmpty())
        {
            const SirilQueryGeometry geometry = sirilQueryGeometry(settings, imageSize, captureDateTimeUtc);
            applyNamedAliasesToCatalog(settings, context.catalogStars);
            mergeBundledBrightStarsIntoCatalog(
                settings,
                context.catalogStars,
                effectiveCatalogLoadMaxMagnitude,
                geometry.valid ? geometry.centerRaDegrees : std::numeric_limits<double>::quiet_NaN(),
                geometry.valid ? geometry.centerDecDegrees : std::numeric_limits<double>::quiet_NaN(),
                geometry.valid ? geometry.queryRadiusDegrees : -1.0);
        }
        if (context.catalogStars.size() < settings.m_plateSolveMinMatches)
        {
            qWarning() << "CameraPlateSolver: Siril SPCC Gaia catalog did not provide enough stars, falling back to HYG/bundled"
                       << "stars" << context.catalogStars.size()
                       << "minMatches" << settings.m_plateSolveMinMatches;
            context.catalogStars.clear();
            context.catalogSource.clear();
        }
    }

    if (context.catalogStars.isEmpty())
    {
        context.catalogStars = brightStarCatalog(settings);
        context.catalogSource = currentCatalogSource(settings);
    }

    populateVisibleCatalogContext(context, settings, captureDateTimeUtc, maxMagnitude, true);
    {
        QMutexLocker locker(&s_contextCacheMutex);
        s_contextCache.append(CatalogContextCacheEntry{cacheKey, context});
        while (s_contextCache.size() > kMaxCachedContexts) {
            s_contextCache.removeFirst();
        }
    }
    return context;
}

void CameraPlateSolver::SolverContext::rebuildVisibleCatalogContext(PlateSolveCatalogContext& context, const CameraSettings& settings, const QDateTime& captureDateTimeUtc, double maxMagnitude)
{
    populateVisibleCatalogContext(context, settings, captureDateTimeUtc, maxMagnitude, true);
}

QVector<CameraPlateSolver::SolverContext::GuidedAnchorPair> CameraPlateSolver::SolverContext::findGuidedAnchorPairs(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<VisibleCatalogStar>& localVisibleStars)
{
    QVector<GuidedAnchorPair> anchors;
    const bool useStartDirection = plateSolveStartUsesDirection(settings);
    const bool useStartRoll = plateSolveStartUsesRoll(settings);
    const bool useWidePlateSolve = isWidePlateSolveContext(settings);
    const bool useFaintNarrowAnchors = useStartDirection
        && !useWidePlateSolve
        && (isNarrowField(settings));
    const bool useWideWeakAnchorSearch = !useStartDirection
        && useWidePlateSolve;
    if ((!useStartDirection && !useWideWeakAnchorSearch)
        || starDetections.isEmpty()
        || localVisibleStars.isEmpty())
    {
        return anchors;
    }

    const bool useStartLens = plateSolveStartUsesLens(settings);
    const SkyProjector currentProjector = createProjector(
        settings,
        imageSize,
        settings.m_azimuth,
        settings.m_elevation,
        settings.m_roll,
        settings.m_fov,
        useStartLens ? settings.m_lensCenterOffsetX : 0.0,
        useStartLens ? settings.m_lensCenterOffsetY : 0.0,
        useStartLens ? settings.m_lensDistortionK1 : 0.0);
    if (!currentProjector.valid) {
        return anchors;
    }

    const QPointF center = projectorPrincipalPoint(currentProjector);
    const double maxImageDimension = std::max(imageSize.width(), imageSize.height());
    const double radialTolerancePixels = useWideWeakAnchorSearch
        ? maxImageDimension
        : std::max(
            static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 6.0,
            maxImageDimension * 0.08);
    const double directDistanceLimitPixels = useWideWeakAnchorSearch
        ? maxImageDimension * 1.5
        : (useFaintNarrowAnchors && !useStartRoll)
            ? maxImageDimension * 1.5
        : std::max(
            radialTolerancePixels * 3.0,
            maxImageDimension * 0.35);
    const double anchorMaxMagnitude = std::min(
        settings.m_plateSolveMaxMagnitude,
        useFaintNarrowAnchors ? 10.0 : 7.0);
    QVector<int> allDetectionIndices;
    allDetectionIndices.reserve(starDetections.size());
    for (int i = 0; i < starDetections.size(); ++i) {
        allDetectionIndices.append(i);
    }
    const QVector<double>& detectionRanks = detectionBrightnessRanks(starDetections, allDetectionIndices);

    for (const VisibleCatalogStar& visibleStar : localVisibleStars)
    {
        if ((visibleStar.catalogIndex < 0)
            || (visibleStar.catalogIndex >= catalogContext.catalogStars.size())
            || (visibleStar.magnitude > anchorMaxMagnitude))
        {
            continue;
        }

        QPointF projectedPoint;
        if (!projectVector(currentProjector, visibleStar.vector, projectedPoint)) {
            continue;
        }

        const double projectedRadius = pointDistancePixels(projectedPoint, center);
        QVector<GuidedAnchorPair> starAnchors;
        starAnchors.reserve(3);
        for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex)
        {
            const CameraPipelineStarDetection& detection = starDetections[detectionIndex];
            if (detection.m_hotPixelSuspect) {
                continue;
            }

            const double reliability = cachedDetectionReliabilityMetric(starDetections, detectionIndex);
            if (reliability <= 0.0) {
                continue;
            }

            const double detectionBrightnessRank = ((detectionIndex >= 0) && (detectionIndex < detectionRanks.size()))
                ? detectionRanks[detectionIndex]
                : 1.0;
            const bool brightCatalogAnchor = useFaintNarrowAnchors
                ? (visibleStar.magnitude <= 10.0)
                : (visibleStar.magnitude <= 5.0);
            const bool brightDetectionAnchor = brightCatalogAnchor
                && (detectionBrightnessRank <= 0.25)
                && !detection.m_hotPixelSuspect;
            const double detectionRadius = pointDistancePixels(detection.m_center, center);
            const double radialError = std::fabs(detectionRadius - projectedRadius);
            const double directDistance = pointDistancePixels(detection.m_center, projectedPoint);
            if (!brightDetectionAnchor
                && (radialError > radialTolerancePixels)
                && (directDistance > radialTolerancePixels))
            {
                continue;
            }
            if (directDistance > directDistanceLimitPixels) {
                continue;
            }

            const QLineF projectedLine(center, projectedPoint);
            const QLineF detectionLine(center, detection.m_center);
            double estimatedRoll = settings.m_roll + projectedLine.angleTo(detectionLine);
            if (!std::isfinite(estimatedRoll)) {
                estimatedRoll = settings.m_roll;
            }

            starAnchors.append({
                detectionIndex,
                visibleStar.catalogIndex,
                radialError,
                directDistance,
                normalizeDegrees(estimatedRoll),
                visibleStar.magnitude,
                reliability,
                detectionBrightnessRank,
                useFaintNarrowAnchors ? narrowGuidedAnchorShapeScore(detection) : 0.0
            });
        }

        std::sort(starAnchors.begin(), starAnchors.end(), [useFaintNarrowAnchors, useStartRoll](const GuidedAnchorPair& lhs, const GuidedAnchorPair& rhs) {
            const bool lhsBright = useFaintNarrowAnchors ? (lhs.magnitude <= 10.0) : (lhs.magnitude <= 5.0);
            const bool rhsBright = useFaintNarrowAnchors ? (rhs.magnitude <= 10.0) : (rhs.magnitude <= 5.0);
            const auto scoreAnchor = [useFaintNarrowAnchors, useStartRoll](const GuidedAnchorPair& anchor, bool bright) {
                if (useFaintNarrowAnchors && !useStartRoll)
                {
                    return anchor.initialDistancePixels * (bright ? 0.05 : 0.18)
                        + anchor.radialErrorPixels * (bright ? 0.03 : 0.10)
                        + (bright ? anchor.detectionBrightnessRank * 60.0 : anchor.detectionBrightnessRank * 120.0)
                        - std::min(120.0, std::log1p(anchor.detectionReliability) * 10.0)
                        - anchor.detectionShapeScore * (bright ? 2.0 : 1.0);
                }

                return bright
                    ? anchor.initialDistancePixels * (useFaintNarrowAnchors ? 0.35 : 0.15)
                        + anchor.radialErrorPixels * (useFaintNarrowAnchors ? 0.15 : 0.05)
                        + anchor.detectionBrightnessRank * (useFaintNarrowAnchors ? 80.0 : 500.0)
                        - std::min(80.0, std::log1p(anchor.detectionReliability) * 8.0)
                        - (useFaintNarrowAnchors ? anchor.detectionShapeScore : 0.0)
                    : anchor.initialDistancePixels
                        + anchor.radialErrorPixels * 0.25
                        - std::min(80.0, std::log1p(anchor.detectionReliability) * 8.0);
            };
            const double lhsScore = scoreAnchor(lhs, lhsBright);
            const double rhsScore = scoreAnchor(rhs, rhsBright);
            if (!qFuzzyCompare(lhsScore + 1.0, rhsScore + 1.0)) {
                return lhsScore < rhsScore;
            }
            return lhs.magnitude < rhs.magnitude;
        });
        const int maxAnchorsForStar = (useFaintNarrowAnchors && !useStartRoll)
            ? (visibleStar.magnitude <= 10.0 ? 16 : 6)
            : (visibleStar.magnitude <= (useFaintNarrowAnchors ? 10.0 : 5.0) ? 6 : 3);
        while (starAnchors.size() > maxAnchorsForStar) {
            starAnchors.removeLast();
        }
        anchors += starAnchors;
    }

    std::sort(anchors.begin(), anchors.end(), [useFaintNarrowAnchors, useStartRoll](const GuidedAnchorPair& lhs, const GuidedAnchorPair& rhs) {
        const bool lhsBright = useFaintNarrowAnchors ? (lhs.magnitude <= 10.0) : (lhs.magnitude <= 5.0);
        const bool rhsBright = useFaintNarrowAnchors ? (rhs.magnitude <= 10.0) : (rhs.magnitude <= 5.0);
        const auto scoreAnchor = [useFaintNarrowAnchors, useStartRoll](const GuidedAnchorPair& anchor, bool bright) {
            if (useFaintNarrowAnchors && !useStartRoll)
            {
                return anchor.magnitude * 45.0
                    + anchor.initialDistancePixels * (bright ? 0.05 : 0.18)
                    + anchor.radialErrorPixels * (bright ? 0.03 : 0.10)
                    + (bright ? anchor.detectionBrightnessRank * 60.0 : anchor.detectionBrightnessRank * 120.0)
                    - std::min(120.0, std::log1p(anchor.detectionReliability) * 10.0)
                    - anchor.detectionShapeScore * (bright ? 2.0 : 1.0);
            }

            return anchor.magnitude * 45.0
                + (bright ? anchor.initialDistancePixels * (useFaintNarrowAnchors ? 0.35 : 0.15) : anchor.initialDistancePixels)
                + (bright ? anchor.radialErrorPixels * (useFaintNarrowAnchors ? 0.15 : 0.05) : anchor.radialErrorPixels * 0.25)
                + (bright ? anchor.detectionBrightnessRank * (useFaintNarrowAnchors ? 80.0 : 500.0) : 0.0)
                - std::min(100.0, std::log1p(anchor.detectionReliability) * 10.0)
                - (useFaintNarrowAnchors ? anchor.detectionShapeScore : 0.0);
        };
        const double lhsScore = scoreAnchor(lhs, lhsBright);
        const double rhsScore = scoreAnchor(rhs, rhsBright);
        if (!qFuzzyCompare(lhsScore + 1.0, rhsScore + 1.0)) {
            return lhsScore < rhsScore;
        }
        if (lhs.catalogIndex != rhs.catalogIndex) {
            return lhs.catalogIndex < rhs.catalogIndex;
        }
        return lhs.detectionIndex < rhs.detectionIndex;
    });
    const int anchorPoolLimit = useWideWeakAnchorSearch ? 96
        : (useFaintNarrowAnchors && !useStartRoll) ? 192
        : useFaintNarrowAnchors ? 96
        : 24;
    while (anchors.size() > anchorPoolLimit) {
        anchors.removeLast();
    }

    return anchors;
}

double CameraPlateSolver::SolverContext::signatureReliabilityScore(const QVector<CameraPipelineStarDetection>& starDetections, const int *indices, int count)
{
    double score = 0.0;
    for (int i = 0; i < count; ++i)
    {
        const int detectionIndex = indices[i];
        if ((detectionIndex < 0) || (detectionIndex >= starDetections.size())) {
            continue;
        }
        score += std::log1p(cachedDetectionReliabilityMetric(starDetections, detectionIndex));
    }
    return score / std::max(1, count);
}

double CameraPlateSolver::SolverContext::signatureBrightnessScore(const QVector<CameraPipelineStarDetection>& starDetections, const int *indices, int count)
{
    double score = 0.0;
    for (int i = 0; i < count; ++i)
    {
        const int detectionIndex = indices[i];
        if ((detectionIndex < 0) || (detectionIndex >= starDetections.size())) {
            continue;
        }
        score += std::log1p(cachedDetectionBrightnessMetric(starDetections, detectionIndex));
    }
    return score / std::max(1, count);
}

bool CameraPlateSolver::SolverContext::isStrongBlindSeedEvaluation(const CameraSettings& settings, const QVector<int>& detectionIndices, const Evaluation& candidate)
{
    if (!candidate.valid) {
        return false;
    }

    const int minBlindSeedMatches = (isNarrowGuidedDirectionSolve(settings))
        ? std::max(settings.m_plateSolveMinMatches, 4)
        : std::max(settings.m_plateSolveMinMatches + 1,
            std::min(6, static_cast<int>(detectionIndices.size())));
    if (candidate.matchCount < minBlindSeedMatches) {
        return false;
    }

    const double medianError = medianCandidateDistancePixels(candidate.matches);
    // Score blind seeds against the *final* (tighter) acceptance radius — the acquisition
    // radius is intentionally generous so that buildMatches finds candidates, but a strong
    // seed must already be well-localised relative to where the final solver will accept.
    const double seedRadius = std::min(
        static_cast<double>(settings.m_plateSolveMatchRadius),
        static_cast<double>(settings.m_plateSolveFinalMatchRadius));
    const double maxRmsError = std::min(seedRadius * 0.60, kBlindSeedMaxRmsPixels);
    const double maxMedianError = std::min(seedRadius * 0.50, kBlindSeedMaxMedianPixels);
    return (candidate.rmsErrorPixels <= maxRmsError) && (medianError <= maxMedianError);
}

CameraPlateSolver::SolverContext::Evaluation CameraPlateSolver::SolverContext::verifyBlindSeedCandidate(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const Evaluation& candidate)
{
    if (!isStrongBlindSeedEvaluation(settings, detectionIndices, candidate)) {
        return Evaluation{};
    }

    int outlierCount = 0;
    const QVector<Match> inlierMatches = rejectOutlierMatches(
        candidate.matches,
        std::max(settings.m_plateSolveMinMatches, 4),
        std::min(settings.m_plateSolveMatchRadius, 12.0),
        &outlierCount);

    const int minConsensusMatches = (isNarrowGuidedDirectionSolve(settings))
        ? std::max(settings.m_plateSolveMinMatches, 4)
        : std::max(settings.m_plateSolveMinMatches + 1, std::min(6, static_cast<int>(detectionIndices.size())));
    if (inlierMatches.size() < minConsensusMatches) {
        return Evaluation{};
    }

    Evaluation consensusCandidate = candidate;
    consensusCandidate.matches = inlierMatches;
    consensusCandidate.matchCount = inlierMatches.size();
    double sumSquaredError = 0.0;
    for (const Match& match : inlierMatches) {
        sumSquaredError += match.distancePixels * match.distancePixels;
    }
    consensusCandidate.rmsErrorPixels = std::sqrt(sumSquaredError / consensusCandidate.matchCount);
    consensusCandidate.valid = true;

    Evaluation refinedCandidate = refinePoseFromMatches(
        settings,
        catalogContext,
        imageSize,
        captureDateTimeUtc,
        starDetections,
        consensusCandidate);

    if (!isStrongBlindSeedEvaluation(settings, detectionIndices, refinedCandidate)) {
        return Evaluation{};
    }

    return refinedCandidate;
}

QVector<CameraPlateSolver::SolverContext::Evaluation> CameraPlateSolver::SolverContext::selectConsensusSeedRepresentatives(const CameraSettings& settings, const QVector<Evaluation>& seeds, int seedLimit, const char *profilePrefix)
{
    if (seeds.size() <= std::max(1, seedLimit)) {
        return seeds;
    }

    const double azElThreshold = std::clamp(
        static_cast<double>(settings.m_fov) * 0.12,
        0.25,
        plateSolveStartUsesDirection(settings) ? 2.0 : 6.0);
    const double rollThreshold = plateSolveStartUsesRoll(settings)
        ? std::max(4.0, std::min(12.0, static_cast<double>(settings.m_fov) * 0.20))
        : 15.0;
    const double fovThreshold = std::clamp(
        static_cast<double>(settings.m_fov) * 0.10,
        0.10,
        8.0);

    QVector<SeedConsensusBasin> basins;
    basins.reserve(std::min<int>(static_cast<int>(seeds.size()), 64));
    for (const Evaluation& seed : seeds)
    {
        if (!seed.valid) {
            continue;
        }

        int basinIndex = -1;
        for (int i = 0; i < basins.size(); ++i)
        {
            const SeedConsensusBasin& basin = basins[i];
            if ((seedWrappedAngularDistanceDegrees(seed.azimuthDegrees, basin.azimuthDegrees) <= azElThreshold)
                && (std::fabs(seed.elevationDegrees - basin.elevationDegrees) <= azElThreshold)
                && (seedWrappedAngularDistanceDegrees(seed.rollDegrees, basin.rollDegrees) <= rollThreshold)
                && (std::fabs(seed.fovDegrees - basin.fovDegrees) <= fovThreshold))
            {
                basinIndex = i;
                break;
            }
        }

        if (basinIndex < 0)
        {
            SeedConsensusBasin basin;
            basin.azimuthDegrees = seed.azimuthDegrees;
            basin.elevationDegrees = seed.elevationDegrees;
            basin.rollDegrees = seed.rollDegrees;
            basin.fovDegrees = seed.fovDegrees;
            basins.append(basin);
            basinIndex = basins.size() - 1;
        }

        SeedConsensusBasin& basin = basins[basinIndex];
        basin.candidates.append(seed);
        const double count = static_cast<double>(basin.candidates.size());
        basin.azimuthDegrees += (seed.azimuthDegrees - basin.azimuthDegrees) / count;
        basin.elevationDegrees += (seed.elevationDegrees - basin.elevationDegrees) / count;
        basin.rollDegrees += (seed.rollDegrees - basin.rollDegrees) / count;
        basin.fovDegrees += (seed.fovDegrees - basin.fovDegrees) / count;
        for (const Match& match : seed.matches)
        {
            basin.detectionSupport.insert(match.detectionIndex);
            basin.catalogSupport.insert(match.catalogIndex);
        }
    }

    for (SeedConsensusBasin& basin : basins)
    {
        std::sort(basin.candidates.begin(), basin.candidates.end(), [](const Evaluation& lhs, const Evaluation& rhs) {
            return seedCandidateConsensusScore(lhs) > seedCandidateConsensusScore(rhs);
        });
        const double independentSupport = static_cast<double>(
            std::min<int>(
                static_cast<int>(basin.detectionSupport.size()),
                static_cast<int>(basin.catalogSupport.size())));
        basin.score = basin.candidates.size() * 2.0
            + independentSupport * 3.0
            + (basin.candidates.isEmpty() ? 0.0 : seedCandidateConsensusScore(basin.candidates.first()));
    }

    std::sort(basins.begin(), basins.end(), [](const SeedConsensusBasin& lhs, const SeedConsensusBasin& rhs) {
        if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
            return lhs.score > rhs.score;
        }
        return lhs.candidates.size() > rhs.candidates.size();
    });

    QVector<Evaluation> selected;
    selected.reserve(seedLimit);
    for (const SeedConsensusBasin& basin : basins)
    {
        const int takeCount = std::min<int>(
            static_cast<int>(basin.candidates.size()),
            std::max(1, seedLimit / std::max(1, std::min(4, static_cast<int>(basins.size())))));
        for (int i = 0; (i < takeCount) && (selected.size() < seedLimit); ++i) {
            selected.append(basin.candidates[i]);
        }
        if (selected.size() >= seedLimit) {
            break;
        }
    }

    recordProfileMetric(QStringLiteral("search.%1ConsensusBasins").arg(QString::fromUtf8(profilePrefix)), basins.size());
    recordProfileMetric(QStringLiteral("search.%1ConsensusSelected").arg(QString::fromUtf8(profilePrefix)), selected.size());
    return selected;
}

QVector<CameraPlateSolver::SolverContext::TriangleSignature> CameraPlateSolver::SolverContext::buildDetectionTriangleSignatures(const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, int maxDetectionCount)
{
    QVector<TriangleSignature> signatures;
    const int maxDetections = std::min<int>(maxDetectionCount, static_cast<int>(detectionIndices.size()));
    for (int i = 0; i < maxDetections; ++i)
    {
        for (int j = i + 1; j < maxDetections; ++j)
        {
            for (int k = j + 1; k < maxDetections; ++k)
            {
                const std::array<int, 3> indices {{
                    detectionIndices[i],
                    detectionIndices[j],
                    detectionIndices[k]
                }};
                const std::array<QPointF, 3> points {{
                    starDetections[indices[0]].m_center,
                    starDetections[indices[1]].m_center,
                    starDetections[indices[2]].m_center
                }};
                TriangleSignature signature = buildTriangleSignature(points);
                if ((signature.longestDistance < 20.0) || !isDistinctiveTriangleSignature(signature)) {
                    continue;
                }
                signature.indices = indices;
                signature.reliabilityScore = signatureReliabilityScore(starDetections, signature.indices.data(), 3);
                signature.brightnessScore = signatureBrightnessScore(starDetections, signature.indices.data(), 3);
                signatures.append(signature);
            }
        }
    }
    std::sort(signatures.begin(), signatures.end(), [](const TriangleSignature& lhs, const TriangleSignature& rhs) {
        const double lhsScore = lhs.areaScore * 2.0 + lhs.reliabilityScore + lhs.brightnessScore * 0.25;
        const double rhsScore = rhs.areaScore * 2.0 + rhs.reliabilityScore + rhs.brightnessScore * 0.25;
        return lhsScore > rhsScore;
    });
    return signatures;
}

bool CameraPlateSolver::SolverContext::appendCatalogTriangleSignature(const CameraSettings& settings, const QVector<VisibleCatalogStar>& visibleStars, int i, int j, int k, QVector<TriangleSignature>& signatures)
{
    const SkyVector& va = visibleStars[i].vector;
    const SkyVector& vb = visibleStars[j].vector;
    const SkyVector& vc = visibleStars[k].vector;
    const SkyVector center = normalize({va.x + vb.x + vc.x,
                                        va.y + vb.y + vc.y,
                                        va.z + vb.z + vc.z});
    if (length(center) <= 0.0) {
        return false;
    }

    // Use the synthetic projector for both ratios *and* orientation so the sign
    // convention matches the detection-side 2D cross product. Ratios computed in
    // pure angular space would be projection-independent, but then the orientation
    // sign would have to match a y-flip convention that is fragile to changes in
    // the SkyProjector implementation. Both ratios and orientation come from the
    // same projected points here so the signs are guaranteed to match the detection
    // triangles built from real image coordinates.
    const double centerAzimuth = normalizeDegrees(std::atan2(center.x, center.y) * 180.0 / kPi);
    const double centerElevation = std::asin(std::clamp(center.z, -1.0, 1.0)) * 180.0 / kPi;
    // Pick a synthetic FoV broad enough to encompass the actual triangle's angular
    // span. Previously this used `max(20, settings.m_fov)`, which under-projected
    // wide-field triangles (forcing them through edge distortion) and biased the
    // ratios. Compute the maximum pairwise angle so we can scale appropriately.
    const double maxPairAngleRad = std::max({
        std::acos(std::clamp(dot(va, vb), -1.0, 1.0)),
        std::acos(std::clamp(dot(va, vc), -1.0, 1.0)),
        std::acos(std::clamp(dot(vb, vc), -1.0, 1.0))
    });
    const double maxPairAngleDeg = maxPairAngleRad * 180.0 / kPi;
    const double syntheticFov = std::clamp(
        std::max(maxPairAngleDeg * 2.5, static_cast<double>(settings.m_fov)),
        20.0,
        160.0);
    const SkyProjector localProjector = createProjector(
        settings,
        QSize(1000, 1000),
        centerAzimuth,
        centerElevation,
        0.0,
        syntheticFov);
    if (!localProjector.valid) {
        return false;
    }

    std::array<QPointF, 3> points;
    const std::array<int, 3> starIndices {{i, j, k}};
    for (int pointIndex = 0; pointIndex < 3; ++pointIndex)
    {
        if (!projectVector(localProjector,
                           visibleStars[starIndices[pointIndex]].vector,
                           points[pointIndex]))
        {
            return false;
        }
    }

    TriangleSignature signature = buildTriangleSignature(points);
    if ((signature.longestDistance < 10.0) || !isDistinctiveTriangleSignature(signature)) {
        return false;
    }
    signature.indices = {{i, j, k}};
    signature.reliabilityScore = signature.areaScore;
    signature.brightnessScore = catalogSignatureBrightnessScore(visibleStars, signature.indices.data(), 3);
    signatures.append(signature);
    return true;
}

QVector<CameraPlateSolver::SolverContext::TriangleSignature> CameraPlateSolver::SolverContext::buildLocalCatalogTriangleSignatures(const CameraSettings& settings, const QVector<VisibleCatalogStar>& visibleStars)
{
    QVector<TriangleSignature> signatures;
    if (visibleStars.size() < 3) {
        return signatures;
    }

    const double binSizeDegrees = std::clamp(static_cast<double>(settings.m_fov) * 2.5, 2.0, 6.0);
    const int azimuthBinCount = std::max(1, static_cast<int>(std::ceil(360.0 / binSizeDegrees)));
    const int elevationBinCount = std::max(1, static_cast<int>(std::ceil(90.0 / binSizeDegrees)));
    QVector<QVector<int>> bins(azimuthBinCount * elevationBinCount);
    const auto binIndex = [azimuthBinCount, elevationBinCount, binSizeDegrees](double azimuthDegrees, double elevationDegrees) {
        const int azimuthBin = std::clamp(
            static_cast<int>(std::floor(normalizeDegrees(azimuthDegrees) / binSizeDegrees)),
            0,
            azimuthBinCount - 1);
        const int elevationBin = std::clamp(
            static_cast<int>(std::floor(std::clamp(elevationDegrees, 0.0, 89.999999) / binSizeDegrees)),
            0,
            elevationBinCount - 1);
        return elevationBin * azimuthBinCount + azimuthBin;
    };

    for (int i = 0; i < visibleStars.size(); ++i) {
        bins[binIndex(visibleStars[i].azimuthDegrees, visibleStars[i].elevationDegrees)].append(i);
    }

    QSet<qint64> seen;
    const int maxLocalStars = 10;
    for (int elevationBin = 0; elevationBin < elevationBinCount; ++elevationBin)
    {
        for (int azimuthBin = 0; azimuthBin < azimuthBinCount; ++azimuthBin)
        {
            QVector<int> localStars;
            localStars.reserve(maxLocalStars * 4);
            for (int elevationOffset = -1; elevationOffset <= 1; ++elevationOffset)
            {
                const int neighborElevationBin = elevationBin + elevationOffset;
                if ((neighborElevationBin < 0) || (neighborElevationBin >= elevationBinCount)) {
                    continue;
                }
                for (int azimuthOffset = -1; azimuthOffset <= 1; ++azimuthOffset)
                {
                    const int neighborAzimuthBin = (azimuthBin + azimuthOffset + azimuthBinCount) % azimuthBinCount;
                    const QVector<int>& cellStars = bins[neighborElevationBin * azimuthBinCount + neighborAzimuthBin];
                    const int takeCount = std::min(maxLocalStars, static_cast<int>(cellStars.size()));
                    for (int i = 0; i < takeCount; ++i) {
                        localStars.append(cellStars[i]);
                    }
                }
            }

            std::sort(localStars.begin(), localStars.end(), [&visibleStars](int lhs, int rhs) {
                return visibleStars[lhs].magnitude < visibleStars[rhs].magnitude;
            });
            QSet<int> localSeen;
            QVector<int> uniqueLocalStars;
            uniqueLocalStars.reserve(localStars.size());
            for (int starIndex : localStars)
            {
                if (localSeen.contains(starIndex)) {
                    continue;
                }
                localSeen.insert(starIndex);
                uniqueLocalStars.append(starIndex);
            }
            localStars = uniqueLocalStars;
            if (localStars.size() > maxLocalStars) {
                localStars.resize(maxLocalStars);
            }

            for (int i = 0; i < localStars.size(); ++i)
            {
                for (int j = i + 1; j < localStars.size(); ++j)
                {
                    for (int k = j + 1; k < localStars.size(); ++k)
                    {
                        const int starI = localStars[i];
                        const int starJ = localStars[j];
                        const int starK = localStars[k];
                        const qint64 key = catalogTriangleKey(starI, starJ, starK);
                        if (seen.contains(key)) {
                            continue;
                        }
                        seen.insert(key);
                        appendCatalogTriangleSignature(settings, visibleStars, starI, starJ, starK, signatures);
                    }
                }
            }
        }
    }

    std::sort(signatures.begin(), signatures.end(), [](const TriangleSignature& lhs, const TriangleSignature& rhs) {
        const double lhsScore = lhs.areaScore * 2.0 + lhs.brightnessScore;
        const double rhsScore = rhs.areaScore * 2.0 + rhs.brightnessScore;
        return lhsScore > rhsScore;
    });
    return signatures;
}

QVector<CameraPlateSolver::SolverContext::TriangleSignature> CameraPlateSolver::SolverContext::buildCatalogTriangleSignatures(const CameraSettings& settings, const QVector<VisibleCatalogStar>& visibleStars, int maxCatalogStarCount)
{
    if (plateSolveStartUsesFov(settings)
        && !plateSolveStartUsesElevation(settings)
        && !plateSolveStartUsesDirection(settings)
        && !isWidePlateSolveContext(settings)
        && (settings.m_fov < 15.0))
    {
        return buildLocalCatalogTriangleSignatures(settings, visibleStars);
    }

    QVector<TriangleSignature> signatures;
    const int requestedMaxCatalogStars = (maxCatalogStarCount > 0) ? maxCatalogStarCount : 32;
    const int maxCatalogStars = std::min<int>(requestedMaxCatalogStars, static_cast<int>(visibleStars.size()));
    for (int i = 0; i < maxCatalogStars; ++i)
    {
        for (int j = i + 1; j < maxCatalogStars; ++j)
        {
            for (int k = j + 1; k < maxCatalogStars; ++k)
            {
                appendCatalogTriangleSignature(settings, visibleStars, i, j, k, signatures);
            }
        }
    }
    std::sort(signatures.begin(), signatures.end(), [](const TriangleSignature& lhs, const TriangleSignature& rhs) {
        const double lhsScore = lhs.areaScore * 2.0 + lhs.brightnessScore;
        const double rhsScore = rhs.areaScore * 2.0 + rhs.brightnessScore;
        return lhsScore > rhsScore;
    });
    return signatures;
}

QVector<CameraPlateSolver::SolverContext::QuadSignature> CameraPlateSolver::SolverContext::buildDetectionQuadSignatures(const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, int maxDetectionCount)
{
    QVector<QuadSignature> signatures;
    const int maxDetections = std::min<int>(maxDetectionCount, static_cast<int>(detectionIndices.size()));
    for (int i = 0; i < maxDetections; ++i)
    {
        for (int j = i + 1; j < maxDetections; ++j)
        {
            for (int k = j + 1; k < maxDetections; ++k)
            {
                for (int l = k + 1; l < maxDetections; ++l)
                {
                    const std::array<int, 4> indices {{
                        detectionIndices[i],
                        detectionIndices[j],
                        detectionIndices[k],
                        detectionIndices[l]
                    }};
                    const std::array<QPointF, 4> points {{
                        starDetections[indices[0]].m_center,
                        starDetections[indices[1]].m_center,
                        starDetections[indices[2]].m_center,
                        starDetections[indices[3]].m_center
                    }};
                    QuadSignature signature = buildQuadSignature(points);
                    if ((signature.longestDistance < 30.0) || !isDistinctiveQuadSignature(signature)) {
                        continue;
                    }
                    signature.indices = indices;
                    signature.reliabilityScore = signatureReliabilityScore(starDetections, signature.indices.data(), 4);
                    signature.brightnessScore = signatureBrightnessScore(starDetections, signature.indices.data(), 4);
                    signatures.append(signature);
                }
            }
        }
    }
    std::sort(signatures.begin(), signatures.end(), [](const QuadSignature& lhs, const QuadSignature& rhs) {
        const double lhsScore = lhs.areaScore * 2.0 + lhs.reliabilityScore + lhs.brightnessScore * 0.25;
        const double rhsScore = rhs.areaScore * 2.0 + rhs.reliabilityScore + rhs.brightnessScore * 0.25;
        return lhsScore > rhsScore;
    });
    return signatures;
}

QVector<CameraPlateSolver::SolverContext::QuadSignature> CameraPlateSolver::SolverContext::buildCatalogQuadSignatures(const CameraSettings& settings, const QVector<VisibleCatalogStar>& visibleStars)
{
    QVector<QuadSignature> signatures;
    const int maxCatalogStars = std::min<int>(24, static_cast<int>(visibleStars.size()));
    for (int i = 0; i < maxCatalogStars; ++i)
    {
        for (int j = i + 1; j < maxCatalogStars; ++j)
        {
            for (int k = j + 1; k < maxCatalogStars; ++k)
            {
                for (int l = k + 1; l < maxCatalogStars; ++l)
                {
                    const SkyVector center = normalize({
                        visibleStars[i].vector.x + visibleStars[j].vector.x + visibleStars[k].vector.x + visibleStars[l].vector.x,
                        visibleStars[i].vector.y + visibleStars[j].vector.y + visibleStars[k].vector.y + visibleStars[l].vector.y,
                        visibleStars[i].vector.z + visibleStars[j].vector.z + visibleStars[k].vector.z + visibleStars[l].vector.z
                    });
                    if (length(center) <= 0.0) {
                        continue;
                    }

                    const double centerAzimuth = normalizeDegrees(std::atan2(center.x, center.y) * 180.0 / kPi);
                    const double centerElevation = std::asin(std::clamp(center.z, -1.0, 1.0)) * 180.0 / kPi;
                    // Scale the synthetic projector's FoV to the actual angular span of the
                    // quad so wide-field constellations don't get pushed through projection
                    // distortion at the synthetic edges. See the matching comment in
                    // buildCatalogTriangleSignatures for rationale.
                    const SkyVector& va = visibleStars[i].vector;
                    const SkyVector& vb = visibleStars[j].vector;
                    const SkyVector& vc = visibleStars[k].vector;
                    const SkyVector& vd = visibleStars[l].vector;
                    const double maxPairAngleRad = std::max({
                        std::acos(std::clamp(dot(va, vb), -1.0, 1.0)),
                        std::acos(std::clamp(dot(va, vc), -1.0, 1.0)),
                        std::acos(std::clamp(dot(va, vd), -1.0, 1.0)),
                        std::acos(std::clamp(dot(vb, vc), -1.0, 1.0)),
                        std::acos(std::clamp(dot(vb, vd), -1.0, 1.0)),
                        std::acos(std::clamp(dot(vc, vd), -1.0, 1.0))
                    });
                    const double maxPairAngleDeg = maxPairAngleRad * 180.0 / kPi;
                    const double syntheticFov = std::clamp(
                        std::max(maxPairAngleDeg * 2.5, static_cast<double>(settings.m_fov)),
                        20.0,
                        160.0);
                    const SkyProjector localProjector = createProjector(
                        settings,
                        QSize(1000, 1000),
                        centerAzimuth,
                        centerElevation,
                        0.0,
                        syntheticFov);
                    if (!localProjector.valid) {
                        continue;
                    }

                    std::array<QPointF, 4> points;
                    bool allProjected = true;
                    const std::array<int, 4> starIndices {{i, j, k, l}};
                    for (int pointIndex = 0; pointIndex < 4; ++pointIndex)
                    {
                        if (!projectVector(localProjector,
                                           visibleStars[starIndices[pointIndex]].vector,
                                           points[pointIndex]))
                        {
                            allProjected = false;
                            break;
                        }
                    }
                    if (!allProjected) {
                        continue;
                    }

                    QuadSignature signature = buildQuadSignature(points);
                    if ((signature.longestDistance < 15.0) || !isDistinctiveQuadSignature(signature)) {
                        continue;
                    }
                    signature.indices = {{i, j, k, l}};
                    signature.reliabilityScore = signature.areaScore;
                    signature.brightnessScore = catalogSignatureBrightnessScore(visibleStars, signature.indices.data(), 4);
                    signatures.append(signature);
                }
            }
        }
    }
    std::sort(signatures.begin(), signatures.end(), [](const QuadSignature& lhs, const QuadSignature& rhs) {
        const double lhsScore = lhs.areaScore * 2.0 + lhs.brightnessScore;
        const double rhsScore = rhs.areaScore * 2.0 + rhs.brightnessScore;
        return lhsScore > rhsScore;
    });
    return signatures;
}

QVector<CameraPlateSolver::SolverContext::Evaluation> CameraPlateSolver::SolverContext::buildBlindTriangleSeeds(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const QVector<VisibleCatalogStar>& visibleStars, const QVector<int>* signatureDetectionIndicesOverride, int maxDetectionSignatureCount, int maxCatalogSignatureStars, int requiredAnchorMatches, int seedLimitOverride)
{
    QVector<Evaluation> seeds;
    if (isCancellationRequested() || (visibleStars.size() < settings.m_plateSolveMinMatches)) {
        return seeds;
    }

    const bool isWideFisheyeLens = isWidePlateSolveContext(settings);
    const QVector<int> defaultSignatureDetectionIndices = isWideFisheyeLens
        ? selectDetectionIndicesForBlindSignatures(starDetections, detectionIndices, 12, 12, 20)
        : detectionIndices;
    const QVector<int>& signatureDetectionIndices = signatureDetectionIndicesOverride
        ? *signatureDetectionIndicesOverride
        : defaultSignatureDetectionIndices;
    const int signatureDetectionCount = (maxDetectionSignatureCount > 0)
        ? maxDetectionSignatureCount
        : isWideFisheyeLens ? 20 : 16;
    const QVector<TriangleSignature> detectionTriangles = buildDetectionTriangleSignatures(
        starDetections,
        signatureDetectionIndices,
        signatureDetectionCount);
    if (isCancellationRequested()) {
        return seeds;
    }
    const QVector<TriangleSignature> catalogTriangles = buildCatalogTriangleSignatures(
        settings,
        visibleStars,
        maxCatalogSignatureStars);
    if (isCancellationRequested() || detectionTriangles.isEmpty() || catalogTriangles.isEmpty()) {
        return seeds;
    }
    const bool brightGuidedTriangleMode = signatureDetectionIndicesOverride
        && plateSolveStartUsesDirection(settings)
        && (isNarrowField(settings))
        && !isWideFisheyeLens;
    const double ratioTolerance = brightGuidedTriangleMode
        ? 0.055
        : (isNarrowGuidedDirectionSolve(settings))
        ? 0.08
        : isWideFisheyeLens ? 0.08
        : kBlindSeedRatioTolerance;
    const bool ignoreOrientationHandedness = isWideFisheyeLens
        || (isNarrowGuidedDirectionSolve(settings));
    const QHash<qint64, QVector<int>> catalogTriangleBuckets =
        buildTriangleSignatureBuckets(catalogTriangles, ratioTolerance);
    const int bucketRadius = 1;
    recordProfileMetric(QStringLiteral("search.triangleDetectionSignatures"), detectionTriangles.size());
    recordProfileMetric(QStringLiteral("search.triangleCatalogSignatures"), catalogTriangles.size());

    const int minBlindSeedMatches = std::max(
        settings.m_plateSolveMinMatches + 1,
        std::min(6, static_cast<int>(detectionIndices.size())));
    const bool useStartLens = plateSolveStartUsesLens(settings);
    const double fixedCenterOffsetX = useStartLens ? settings.m_lensCenterOffsetX : 0.0;
    const double fixedCenterOffsetY = useStartLens ? settings.m_lensCenterOffsetY : 0.0;
    const double fixedDistortionK1 = useStartLens ? settings.m_lensDistortionK1 : 0.0;

    struct TriedDirection { double azimuthDegrees; double elevationDegrees; };
    QVector<TriedDirection> triedDirections;
    bool earlyExit = false;
    qint64 ratioCandidates = 0;
    qint64 ratioMatches = 0;
    qint64 seedEvaluations = 0;
    qint64 seedValidEvaluations = 0;
    qint64 seedAnchorSupported = 0;
    int maxSeedAnchorMatches = 0;
    qint64 verifiedSeeds = 0;
    QVector<quint8> baseDetectionSelected(starDetections.size(), 0);
    for (int detectionIndex : detectionIndices)
    {
        if ((detectionIndex >= 0) && (detectionIndex < baseDetectionSelected.size())) {
            baseDetectionSelected[detectionIndex] = 1;
        }
    }

    for (const TriangleSignature& detectionTriangle : detectionTriangles)
    {
        if (earlyExit || isCancellationRequested()) break;
        QVector<int> seedDetectionIndices;
        const QVector<int>* seedDetectionIndicesPtr = &detectionIndices;
        for (int detectionIndex : detectionTriangle.indices)
        {
            if ((detectionIndex < 0)
                || (detectionIndex >= baseDetectionSelected.size())
                || baseDetectionSelected[detectionIndex])
            {
                continue;
            }
            if (seedDetectionIndices.isEmpty()) {
                seedDetectionIndices = detectionIndices;
            }
            if (!seedDetectionIndices.contains(detectionIndex)) {
                seedDetectionIndices.append(detectionIndex);
            }
        }
        if (!seedDetectionIndices.isEmpty()) {
            seedDetectionIndicesPtr = &seedDetectionIndices;
        }

        const int detectionShortBin = signatureRatioBin(detectionTriangle.ratioShortToLong, ratioTolerance);
        const int detectionMidBin = signatureRatioBin(detectionTriangle.ratioMidToLong, ratioTolerance);
        for (int shortBinOffset = -bucketRadius; shortBinOffset <= bucketRadius; ++shortBinOffset)
        {
            if (earlyExit || isCancellationRequested()) break;
            for (int midBinOffset = -bucketRadius; midBinOffset <= bucketRadius; ++midBinOffset)
            {
                if (earlyExit || isCancellationRequested()) break;
                const auto bucketIt = catalogTriangleBuckets.constFind(
                    signatureBucketKey(detectionShortBin + shortBinOffset, detectionMidBin + midBinOffset));
                if (bucketIt == catalogTriangleBuckets.constEnd()) {
                    continue;
                }
                for (int catalogTriangleIndex : *bucketIt)
                {
                    if (earlyExit || isCancellationRequested()) break;
                    const TriangleSignature& catalogTriangle = catalogTriangles.at(catalogTriangleIndex);
                    ++ratioCandidates;
                    if (std::fabs(detectionTriangle.ratioShortToLong - catalogTriangle.ratioShortToLong) > ratioTolerance
                        || std::fabs(detectionTriangle.ratioMidToLong - catalogTriangle.ratioMidToLong) > ratioTolerance)
                    {
                        continue;
                    }

                    ++ratioMatches;
                    if (!ignoreOrientationHandedness && ((detectionTriangle.orientation * catalogTriangle.orientation) < 0.0)) {
                        continue;
                    }

                    const VisibleCatalogStar& a = visibleStars[catalogTriangle.indices[0]];
                    const VisibleCatalogStar& b = visibleStars[catalogTriangle.indices[1]];
                    const VisibleCatalogStar& c = visibleStars[catalogTriangle.indices[2]];
            const SkyVector center = normalize({
                a.vector.x + b.vector.x + c.vector.x,
                a.vector.y + b.vector.y + c.vector.y,
                a.vector.z + b.vector.z + c.vector.z
            });
            if (length(center) <= 0.0) {
                continue;
            }

            const double seedAzimuth = brightGuidedTriangleMode
                ? static_cast<double>(settings.m_azimuth)
                : normalizeDegrees(std::atan2(center.x, center.y) * 180.0 / kPi);
            const double seedElevation = brightGuidedTriangleMode
                ? static_cast<double>(settings.m_elevation)
                : std::asin(std::clamp(center.z, -1.0, 1.0)) * 180.0 / kPi;

            // Use the longest pairwise angular distance so the scale estimate matches
            // detectionTriangle.longestDistance regardless of which edge is longest.
            const double abAngle = std::acos(std::clamp(dot(a.vector, b.vector), -1.0, 1.0)) * 180.0 / kPi;
            const double acAngle = std::acos(std::clamp(dot(a.vector, c.vector), -1.0, 1.0)) * 180.0 / kPi;
            const double bcAngle = std::acos(std::clamp(dot(b.vector, c.vector), -1.0, 1.0)) * 180.0 / kPi;
            const double catalogAngularDistance = std::max({abAngle, acAngle, bcAngle});
            if (catalogAngularDistance <= 0.01) {
                continue;
            }

            const double triangleSeedFov = std::clamp(
                catalogAngularDistance * static_cast<double>(std::max(imageSize.width(), imageSize.height())) / std::max(1.0, detectionTriangle.longestDistance),
                static_cast<double>(CameraSettings::m_minFov),
                static_cast<double>(CameraSettings::m_maxFov));
            const double baseSeedFov = brightGuidedTriangleMode
                ? static_cast<double>(settings.m_fov)
                : triangleSeedFov;
            if (!seedFovCompatibleWithStartFov(settings, baseSeedFov)) {
                continue;
            }

            // Skip sky directions already tried by a previous triangle match. The dedup
            // radius scales with the seed FoV: at 90° FoV the original 3° tolerance is fine,
            // but at 15-25° wide-field blind FoVs a 3° basin can swallow the correct
            // direction after a near-miss. Use 5% of seed FoV with a 0.5°-5° clamp.
            if (!brightGuidedTriangleMode)
            {
                const double dedupRadiusDegrees = std::clamp(baseSeedFov * 0.05, 0.5, 5.0);
                bool alreadyTried = false;
                for (const TriedDirection& tried : triedDirections) {
                // Use wrap-aware azimuth distance so seeds near 0°/360° are correctly deduped.
                    const double azDiff = std::fabs(seedAzimuth - tried.azimuthDegrees);
                    const double azDist = (azDiff <= 180.0) ? azDiff : 360.0 - azDiff;
                    if (azDist < dedupRadiusDegrees
                        && std::fabs(seedElevation - tried.elevationDegrees) < dedupRadiusDegrees)
                    {
                        alreadyTried = true;
                        break;
                    }
                }
                if (alreadyTried) continue;
                triedDirections.append({seedAzimuth, seedElevation});
            }

            const std::array<QPointF, 3> detectionPoints {{
                starDetections[detectionTriangle.indices[0]].m_center,
                starDetections[detectionTriangle.indices[1]].m_center,
                starDetections[detectionTriangle.indices[2]].m_center
            }};
            const std::array<VisibleCatalogStar, 3> triangleStars {{a, b, c}};

            QVector<int> allowedCatalogIndices {
                a.catalogIndex,
                b.catalogIndex,
                c.catalogIndex
            };

            // Sweep FoV around the seed estimate. The base estimate is derived from a
            // rectilinear scale model (long_edge_pixels / longest_pixel_distance), which biases
            // the result low for fisheye lenses; widen the sweep accordingly. Also clamp to
            // CameraSettings::m_maxFov for consistency with the base-clamp above — the
            // earlier hard-coded 180.0 ceiling was inconsistent with the rest of the file.
            const bool isFisheyeLens = (settings.m_lensProjection != CameraSettings::LensProjectionRectilinear);
            const std::array<double, 5> fovScales = plateSolveStartUsesFov(settings)
                ? std::array<double, 5>{{0.96, 1.0, 1.04, 1.0, 1.0}}
                : isFisheyeLens
                    ? std::array<double, 5>{{0.60, 0.80, 1.0, 1.25, 1.60}}
                    : std::array<double, 5>{{0.85, 0.93, 1.0, 1.07, 1.15}};
            const int fovScaleCount = plateSolveStartUsesFov(settings) ? 3 : 5;
            const double seedBaseFov = plateSolveStartUsesFov(settings)
                ? static_cast<double>(settings.m_fov)
                : baseSeedFov;
            for (int fovScaleIndex = 0; fovScaleIndex < fovScaleCount; ++fovScaleIndex)
            {
                if (earlyExit || isCancellationRequested()) break;
                const double fovScale = fovScales[fovScaleIndex];
                const double seedFov = std::clamp(
                    seedBaseFov * fovScale,
                    static_cast<double>(CameraSettings::m_minFov),
                    static_cast<double>(CameraSettings::m_maxFov));
                const SkyProjector rollProjector = createProjector(
                    settings,
                    imageSize,
                    seedAzimuth,
                    seedElevation,
                    0.0,
                    seedFov,
                    fixedCenterOffsetX,
                    fixedCenterOffsetY,
                    fixedDistortionK1);
                if (!rollProjector.valid) {
                    continue;
                }

                std::array<QPointF, 3> projectedPoints;
                bool projected = true;
                for (int idx = 0; idx < 3; ++idx)
                {
                    if (!projectVector(rollProjector, triangleStars[idx].vector, projectedPoints[idx]))
                    {
                        projected = false;
                        break;
                    }
                }
                if (!projected) {
                    continue;
                }

                const SkyProjector sourceProjector = createProjector(
                    settings,
                    imageSize,
                    0.0,
                    90.0,
                    0.0,
                    seedFov,
                    fixedCenterOffsetX,
                    fixedCenterOffsetY,
                    fixedDistortionK1);
                SkyVector sourceA;
                SkyVector sourceB;
                std::array<SkyVector, 3> sourceTriangleVectors;
                bool haveSourceTriangle = sourceProjector.valid;
                if (haveSourceTriangle)
                {
                    for (int idx = 0; idx < 3; ++idx)
                    {
                        if (!unprojectPixelToVector(sourceProjector, detectionPoints[idx], sourceTriangleVectors[idx]))
                        {
                            haveSourceTriangle = false;
                            break;
                        }
                    }
                }
                const bool haveSourcePair = sourceProjector.valid
                    && unprojectPixelToVector(sourceProjector, detectionPoints[0], sourceA)
                    && unprojectPixelToVector(sourceProjector, detectionPoints[1], sourceB);

                for (const std::array<int, 3>& permutation : kTrianglePermutations)
                {
                    if (earlyExit || isCancellationRequested()) break;
                    const bool brightnessOrderCompatible = triangleBrightnessOrderCompatible(
                            starDetections,
                            detectionTriangle.indices,
                            triangleStars,
                            permutation);
                    if (brightGuidedTriangleMode && !brightnessOrderCompatible)
                    {
                        continue;
                    }
                    if (!brightGuidedTriangleMode)
                    {
                        const double brightnessMagnitudeError = triangleBrightnessMagnitudeError(
                            starDetections,
                            detectionTriangle.indices,
                            triangleStars,
                            permutation);
                        if (!brightnessOrderCompatible && (brightnessMagnitudeError > 2.75)) {
                            continue;
                        }
                    }

                    QVector<GuidedTrianglePoseSeed> poseSeeds;
                    if (brightGuidedTriangleMode)
                    {
                        poseSeeds = guidedTriangleSimilarityPoseSeeds(
                            settings,
                            imageSize,
                            detectionPoints,
                            triangleStars,
                            permutation,
                            seedAzimuth,
                            seedElevation,
                            seedFov,
                            fixedCenterOffsetX,
                            fixedCenterOffsetY,
                            fixedDistortionK1);
                        if (!haveSourceTriangle) {
                            if (poseSeeds.isEmpty()) {
                                continue;
                            }
                        }
                        else
                        {
                            GuidedTrianglePoseSeed wahbaSeed;
                            wahbaSeed.azimuthDegrees = seedAzimuth;
                            wahbaSeed.elevationDegrees = seedElevation;
                            wahbaSeed.rollDegrees = 0.0;
                            wahbaSeed.fovDegrees = seedFov;
                            const std::array<SkyVector, 3> targetTriangleVectors {{
                                triangleStars[permutation[0]].vector,
                                triangleStars[permutation[1]].vector,
                                triangleStars[permutation[2]].vector
                            }};
                            double triangleRmsAngularError = std::numeric_limits<double>::infinity();
                            double triangleMaxAngularError = std::numeric_limits<double>::infinity();
                            if (poseFromThreeVectorPairs(
                                    sourceProjector,
                                    sourceTriangleVectors,
                                    targetTriangleVectors,
                                    wahbaSeed.azimuthDegrees,
                                    wahbaSeed.elevationDegrees,
                                    wahbaSeed.rollDegrees,
                                    &triangleRmsAngularError,
                                    &triangleMaxAngularError))
                            {
                                Q_UNUSED(triangleRmsAngularError)
                                Q_UNUSED(triangleMaxAngularError)
                                poseSeeds.append(wahbaSeed);
                            }
                        }
                        if (poseSeeds.isEmpty()) {
                            continue;
                        }
                    }
                    else
                    {
                        GuidedTrianglePoseSeed poseSeed;
                        poseSeed.azimuthDegrees = seedAzimuth;
                        poseSeed.elevationDegrees = seedElevation;
                        poseSeed.rollDegrees = 0.0;
                        poseSeed.fovDegrees = seedFov;
                        if (haveSourcePair)
                        {
                            if (!poseFromTwoVectorPairs(
                                    sourceProjector,
                                    sourceA,
                                    sourceB,
                                    triangleStars[permutation[0]].vector,
                                    triangleStars[permutation[1]].vector,
                                    poseSeed.azimuthDegrees,
                                    poseSeed.elevationDegrees,
                                    poseSeed.rollDegrees))
                            {
                                continue;
                            }
                        }
                        else
                        {
                            const QLineF detectionBase(detectionPoints[0], detectionPoints[1]);
                            const QLineF projectedBase(projectedPoints[permutation[0]], projectedPoints[permutation[1]]);
                            poseSeed.rollDegrees = projectedBase.angleTo(detectionBase);
                            if (!std::isfinite(poseSeed.rollDegrees)) {
                                poseSeed.rollDegrees = 0.0;
                            }
                        }
                        poseSeeds.append(poseSeed);
                    }

                    for (const GuidedTrianglePoseSeed& poseSeed : poseSeeds)
                    {
                        if (earlyExit || isCancellationRequested()) break;
                        // Sweep small roll perturbations to tolerate centroiding noise on the reference edge.
                        for (double rollDelta : {-10.0, -5.0, 0.0, 5.0, 10.0})
                        {
                            if (earlyExit || isCancellationRequested()) break;
                            const double seedRoll = poseSeed.rollDegrees + rollDelta;

                            const Evaluation seededCandidate = evaluatePose(
                                settings,
                                catalogContext,
                                imageSize,
                                captureDateTimeUtc,
                                starDetections,
                                *seedDetectionIndicesPtr,
                                poseSeed.azimuthDegrees,
                                poseSeed.elevationDegrees,
                                seedRoll,
                                poseSeed.fovDegrees,
                                &allowedCatalogIndices,
                                fixedCenterOffsetX,
                                fixedCenterOffsetY,
                                fixedDistortionK1);
                            ++seedEvaluations;
                            if (brightGuidedTriangleMode && (seedEvaluations >= 4000)) {
                                earlyExit = true;
                            }
                            if (!seededCandidate.valid) {
                                continue;
                            }
                            ++seedValidEvaluations;
                            const std::array<int, 3> anchorCatalogIndices {{
                                triangleStars[permutation[0]].catalogIndex,
                                triangleStars[permutation[1]].catalogIndex,
                                triangleStars[permutation[2]].catalogIndex
                            }};
                            const double anchorDistancePixels = brightGuidedTriangleMode
                                ? std::min(
                                    128.0,
                                    std::max(
                                        static_cast<double>(settings.m_plateSolveMatchRadius),
                                        static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 4.0))
                                : std::min(
                                    static_cast<double>(settings.m_plateSolveMatchRadius),
                                    std::max(6.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius)));
                            const int seedAnchorMatches = countProjectedAnchorSupport(
                                settings,
                                catalogContext,
                                imageSize,
                                starDetections,
                                seededCandidate,
                                detectionTriangle.indices,
                                anchorCatalogIndices,
                                anchorDistancePixels);
                            maxSeedAnchorMatches = std::max(maxSeedAnchorMatches, seedAnchorMatches);
                            if (seedAnchorMatches < std::min<int>(requiredAnchorMatches, 3)) {
                                continue;
                            }
                            ++seedAnchorSupported;

                            Evaluation candidate = evaluatePose(
                                settings,
                                catalogContext,
                                imageSize,
                                captureDateTimeUtc,
                                starDetections,
                                *seedDetectionIndicesPtr,
                                seededCandidate.azimuthDegrees,
                                seededCandidate.elevationDegrees,
                                seededCandidate.rollDegrees,
                                seededCandidate.fovDegrees,
                                nullptr,
                                fixedCenterOffsetX,
                                fixedCenterOffsetY,
                                fixedDistortionK1);
                            if (brightGuidedTriangleMode && candidate.valid)
                            {
                                candidate.anchored = true;
                                candidate.guidedTriangle = true;
                                candidate.anchorDetectionIndex = detectionTriangle.indices[0];
                                candidate.anchorCatalogIndex = anchorCatalogIndices[0];
                                candidate.secondaryAnchorDetectionIndex = detectionTriangle.indices[1];
                                candidate.secondaryAnchorCatalogIndex = anchorCatalogIndices[1];
                                candidate.tertiaryAnchorDetectionIndex = detectionTriangle.indices[2];
                                candidate.tertiaryAnchorCatalogIndex = anchorCatalogIndices[2];
                                seeds.append(candidate);
                            }
                            const Evaluation verifiedCandidate = verifyBlindSeedCandidate(
                                settings,
                                catalogContext,
                                imageSize,
                                captureDateTimeUtc,
                                starDetections,
                                *seedDetectionIndicesPtr,
                                candidate);
                            if (verifiedCandidate.valid) {
                                seeds.append(verifiedCandidate);
                                ++verifiedSeeds;
                                if (verifiedCandidate.matchCount >= minBlindSeedMatches + 3
                                    && verifiedCandidate.rmsErrorPixels < kBlindSeedMaxRmsPixels * 0.5)
                                {
                                    earlyExit = true;
                                }
                            }
                        }
                        if (brightGuidedTriangleMode && (seedEvaluations >= 4000))
                        {
                            earlyExit = true;
                            break;
                        }
                    }
                }
            }
        }
    }

            }
        }

    recordProfileMetric(QStringLiteral("search.triangleRatioCandidates"), ratioCandidates);
    recordProfileMetric(QStringLiteral("search.triangleRatioMatches"), ratioMatches);
    recordProfileMetric(QStringLiteral("search.triangleSeedEvaluations"), seedEvaluations);
    recordProfileMetric(QStringLiteral("search.triangleSeedValid"), seedValidEvaluations);
    recordProfileMetric(QStringLiteral("search.triangleSeedAnchorSupported"), seedAnchorSupported);
    recordProfileMetric(QStringLiteral("search.triangleSeedMaxAnchors"), maxSeedAnchorMatches);
    recordProfileMetric(QStringLiteral("search.triangleVerifiedSeeds"), verifiedSeeds);

    std::sort(seeds.begin(), seeds.end(), [this](const Evaluation& lhs, const Evaluation& rhs) {
        return isBetterWeakModeEvaluation(lhs, rhs);
    });
    const int seedLimit = (seedLimitOverride > 0)
        ? seedLimitOverride
        : isWideFisheyeLens ? 64 : 16;
    return selectConsensusSeedRepresentatives(settings, seeds, seedLimit, "triangle");
}

QVector<CameraPlateSolver::SolverContext::Evaluation> CameraPlateSolver::SolverContext::buildBrightGuidedTriangleSeeds(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const QVector<VisibleCatalogStar>& visibleStars)
{
    if (isCancellationRequested()
        || !plateSolveStartUsesDirection(settings)
        || isWidePlateSolveContext(settings)
        || (!isNarrowField(settings))
        || (starDetections.size() < 3)
        || (visibleStars.size() < 3))
    {
        return {};
    }

    const QVector<int> triangleDetectionIndices = selectDetectionIndicesForBrightGuidedTriangles(
        starDetections,
        detectionIndices);
    if (triangleDetectionIndices.size() < 3) {
        return {};
    }

    QVector<VisibleCatalogStar> triangleCatalogStars;
    triangleCatalogStars.reserve(std::min(64, static_cast<int>(visibleStars.size())));
    const double maxMagnitude = static_cast<double>(settings.m_plateSolveMaxMagnitude);
    for (const VisibleCatalogStar& visibleStar : visibleStars)
    {
        if (visibleStar.magnitude > maxMagnitude) {
            break;
        }
        triangleCatalogStars.append(visibleStar);
        if (triangleCatalogStars.size() >= 64) {
            break;
        }
    }
    if (triangleCatalogStars.size() < 3) {
        return {};
    }

    if (qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE"))
    {
        qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: bright-triangle pools"
                 << "detections" << triangleDetectionIndices.size()
                 << "catalog" << triangleCatalogStars.size();
    }

    return buildBlindTriangleSeeds(
        settings,
        catalogContext,
        imageSize,
        captureDateTimeUtc,
        starDetections,
        detectionIndices,
        triangleCatalogStars,
        &triangleDetectionIndices,
        32,
        48,
        3,
        48);
}

QVector<CameraPlateSolver::SolverContext::Evaluation> CameraPlateSolver::SolverContext::buildBrightGuidedAnchorTriangleSeeds(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const QVector<VisibleCatalogStar>& visibleStars)
{
    QElapsedTimer anchorTriangleTimer;
    anchorTriangleTimer.start();
    QVector<Evaluation> seeds;
    if (isCancellationRequested()
        || !plateSolveStartUsesDirection(settings)
        || isWidePlateSolveContext(settings)
        || (!isNarrowField(settings))
        || (starDetections.size() < 3)
        || (visibleStars.size() < 3))
    {
        return seeds;
    }

    const bool useStartLens = plateSolveStartUsesLens(settings);
    const double fixedCenterOffsetX = useStartLens ? settings.m_lensCenterOffsetX : 0.0;
    const double fixedCenterOffsetY = useStartLens ? settings.m_lensCenterOffsetY : 0.0;
    const double fixedDistortionK1 = useStartLens ? settings.m_lensDistortionK1 : 0.0;
    const SkyProjector radialProjector = createProjector(
        settings,
        imageSize,
        settings.m_azimuth,
        settings.m_elevation,
        0.0,
        settings.m_fov,
        fixedCenterOffsetX,
        fixedCenterOffsetY,
        fixedDistortionK1);
    if (!radialProjector.valid) {
        return seeds;
    }

    QVector<int> allDetectionIndices;
    allDetectionIndices.reserve(starDetections.size());
    for (int i = 0; i < starDetections.size(); ++i) {
        allDetectionIndices.append(i);
    }
    const QVector<double>& detectionRanks = detectionBrightnessRanks(starDetections, allDetectionIndices);

    struct AnchorCandidate
    {
        int detectionIndex = -1;
        int catalogIndex = -1;
        VisibleCatalogStar catalogStar;
        double score = 0.0;
        double radialErrorPixels = 0.0;
        double magnitude = 0.0;
    };

    const QPointF principalPoint = projectorPrincipalPoint(radialProjector);
    const double maxImageDimension = std::max(imageSize.width(), imageSize.height());
    const double radialTolerancePixels = std::max(
        static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 12.0,
        maxImageDimension * 0.24);
    const double maxCatalogMagnitude = static_cast<double>(settings.m_plateSolveMaxMagnitude);
    const int maxCatalogStars = 120;
    const double maxFrameRadius = std::max(
        std::max(pointDistancePixels(QPointF(0.0, 0.0), principalPoint),
                 pointDistancePixels(QPointF(imageSize.width(), 0.0), principalPoint)),
        std::max(pointDistancePixels(QPointF(0.0, imageSize.height()), principalPoint),
                 pointDistancePixels(QPointF(imageSize.width(), imageSize.height()), principalPoint)));

    struct ProjectedAnchorCatalogStar
    {
        VisibleCatalogStar visibleStar;
        QPointF projectedPoint;
        double projectedRadius = 0.0;
    };
    QVector<ProjectedAnchorCatalogStar> anchorCatalogStars;
    anchorCatalogStars.reserve(maxCatalogStars * 2);
    int anchorCatalogScanned = 0;
    int anchorCatalogProjected = 0;
    for (const VisibleCatalogStar& visibleStar : visibleStars)
    {
        if (isCancellationRequested()) {
            return seeds;
        }
        if (visibleStar.magnitude > maxCatalogMagnitude) {
            break;
        }
        ++anchorCatalogScanned;

        QPointF projectedPoint;
        if (!projectVector(radialProjector, visibleStar.vector, projectedPoint)) {
            continue;
        }
        ++anchorCatalogProjected;

        const double projectedRadius = pointDistancePixels(projectedPoint, principalPoint);
        if (projectedRadius > (maxFrameRadius + radialTolerancePixels)) {
            continue;
        }

        anchorCatalogStars.append({visibleStar, projectedPoint, projectedRadius});
    }
    std::sort(anchorCatalogStars.begin(), anchorCatalogStars.end(), [](const ProjectedAnchorCatalogStar& lhs, const ProjectedAnchorCatalogStar& rhs) {
        if (!qFuzzyCompare(lhs.visibleStar.magnitude + 1.0, rhs.visibleStar.magnitude + 1.0)) {
            return lhs.visibleStar.magnitude < rhs.visibleStar.magnitude;
        }
        if (!qFuzzyCompare(lhs.projectedRadius + 1.0, rhs.projectedRadius + 1.0)) {
            return lhs.projectedRadius < rhs.projectedRadius;
        }
        return lhs.visibleStar.catalogIndex < rhs.visibleStar.catalogIndex;
    });
    if (anchorCatalogStars.size() > maxCatalogStars) {
        anchorCatalogStars.resize(maxCatalogStars);
    }
    recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleCatalogStars"), anchorCatalogStars.size());
    recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleCatalogScanned"), anchorCatalogScanned);
    recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleCatalogProjected"), anchorCatalogProjected);

    QVector<AnchorCandidate> anchors;
    anchors.reserve(maxCatalogStars * 6);
    for (const ProjectedAnchorCatalogStar& projectedCatalogStar : anchorCatalogStars)
    {
        if (isCancellationRequested()) {
            return seeds;
        }
        const VisibleCatalogStar& visibleStar = projectedCatalogStar.visibleStar;

        const bool namedCatalogStar = (visibleStar.catalogIndex >= 0)
            && (visibleStar.catalogIndex < catalogContext.catalogStars.size())
            && isNamedSparseGuidedCatalogStar(catalogContext.catalogStars[visibleStar.catalogIndex]);
        const double projectedRadius = projectedCatalogStar.projectedRadius;
        QVector<AnchorCandidate> perCatalogAnchors;
        perCatalogAnchors.reserve(12);
        for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex)
        {
            const CameraPipelineStarDetection& detection = starDetections[detectionIndex];
            if (detection.m_hotPixelSuspect) {
                continue;
            }

            const double reliability = cachedDetectionReliabilityMetric(starDetections, detectionIndex);
            if (reliability <= 0.0) {
                continue;
            }

            const double shapeScore = narrowGuidedAnchorShapeScore(detection);
            if (!detection.m_saturated
                && (shapeScore < 8.0)
                && (cachedDetectionBrightnessMetric(starDetections, detectionIndex) < 25.0)
                && (reliability < 25.0))
            {
                continue;
            }

            const double radialError = std::fabs(
                pointDistancePixels(detection.m_center, principalPoint) - projectedRadius);
            if (radialError > radialTolerancePixels) {
                continue;
            }

            const double detectionRank = ((detectionIndex >= 0) && (detectionIndex < detectionRanks.size()))
                ? detectionRanks[detectionIndex]
                : 0.5;
            const double score = radialError * 0.20
                + visibleStar.magnitude * 8.0
                + detectionRank * 70.0
                - std::min(90.0, std::log1p(reliability) * 8.0)
                - std::min(60.0, shapeScore * 0.8)
                - (namedCatalogStar ? 18.0 : 0.0);
            perCatalogAnchors.append({
                detectionIndex,
                visibleStar.catalogIndex,
                visibleStar,
                score,
                radialError,
                visibleStar.magnitude
            });
            if (debugCatalogStarMatches(catalogContext, visibleStar.catalogIndex))
            {
                qCDebug(cameraPlateSolverLog).noquote() << "CameraPlateSolver: ANCHOR candidate for" << catalogDisplayName(catalogContext.catalogStars[visibleStar.catalogIndex])
                    << "detection #" << detectionIndex
                    << "center" << detection.m_center
                    << "score" << score
                    << "rank" << detectionRank
                    << "brightness" << cachedDetectionBrightnessMetric(starDetections, detectionIndex)
                    << "reliability" << reliability
                    << "shapeScore" << shapeScore
                    << "radialError" << radialError
                    << "flux" << detection.m_flux
                    << "peak" << detection.m_peakValue
                    << "saturated" << detection.m_saturated
                    << "snr" << detection.m_snr
                    << "fwhm" << detection.m_fwhm;
            }
        }

        std::sort(perCatalogAnchors.begin(), perCatalogAnchors.end(), [](const AnchorCandidate& lhs, const AnchorCandidate& rhs) {
            if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
                return lhs.score < rhs.score;
            }
            return lhs.radialErrorPixels < rhs.radialErrorPixels;
        });
        const int perCatalogLimit = std::min(
            static_cast<int>(perCatalogAnchors.size()),
            visibleStar.magnitude <= 10.0 ? 12 : 8);
        for (int i = 0; i < perCatalogLimit; ++i) {
            anchors.append(perCatalogAnchors[i]);
        }
    }

    QSet<int> uniqueAnchorCatalogs;
    QSet<int> uniqueAnchorDetections;
    for (const AnchorCandidate& anchor : anchors)
    {
        uniqueAnchorCatalogs.insert(anchor.catalogIndex);
        uniqueAnchorDetections.insert(anchor.detectionIndex);
    }
    recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleUniqueCatalogs"), uniqueAnchorCatalogs.size());
    recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleUniqueDetections"), uniqueAnchorDetections.size());

    if ((anchors.size() < 3) || (uniqueAnchorCatalogs.size() < 3) || (uniqueAnchorDetections.size() < 3)) {
        recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleAnchors"), anchors.size());
        return seeds;
    }

    std::sort(anchors.begin(), anchors.end(), [](const AnchorCandidate& lhs, const AnchorCandidate& rhs) {
        if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
            return lhs.score < rhs.score;
        }
        return lhs.magnitude < rhs.magnitude;
    });
    if (anchors.size() > 192) {
        anchors.resize(192);
    }

    const auto sortedRatios = [](std::array<double, 3> distances) {
        std::sort(distances.begin(), distances.end());
        if (distances[2] <= 1e-6) {
            return std::array<double, 3>{{0.0, 0.0, 0.0}};
        }
        return std::array<double, 3>{{
            distances[0] / distances[2],
            distances[1] / distances[2],
            distances[2]
        }};
    };

    struct TriangleCandidate
    {
        std::array<int, 3> anchorIndices{{-1, -1, -1}};
        double score = 0.0;
        double baseFov = 0.0;
    };
    QVector<TriangleCandidate> triangleCandidates;
    triangleCandidates.reserve(1024);
    qint64 ratioCandidates = 0;
    qint64 ratioMatches = 0;
    const double ratioTolerance = 0.085;
    const double startDirectionMaxDelta = std::max(
        static_cast<double>(settings.m_plateSolveAzElSearchRadius) + static_cast<double>(settings.m_fov) * 2.0,
        static_cast<double>(settings.m_fov) * 4.0);

#if 1
    struct RollConsensusDetection
    {
        int detectionIndex = -1;
        double radiusPixels = 0.0;
        double angleDegrees = 0.0;
        double score = 0.0;
        double brightness = 0.0;
        double reliability = 0.0;
        double shapeScore = 0.0;
    };
    QVector<RollConsensusDetection> rollConsensusDetections;
    rollConsensusDetections.reserve(starDetections.size());
    for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex)
    {
        const CameraPipelineStarDetection& detection = starDetections[detectionIndex];
        if (!isDetectionUsableForBrightPrior(detection)) {
            continue;
        }

        const double dx = detection.m_center.x() - principalPoint.x();
        const double dy = principalPoint.y() - detection.m_center.y();
        const double radiusPixels = std::hypot(dx, dy);
        if (radiusPixels < 18.0) {
            continue;
        }

        const double brightness = cachedDetectionBrightnessMetric(starDetections, detectionIndex);
        const double reliability = cachedDetectionReliabilityMetric(starDetections, detectionIndex);
        const double shapeScore = narrowGuidedAnchorShapeScore(detection);
        RollConsensusDetection rollDetection;
        rollDetection.detectionIndex = detectionIndex;
        rollDetection.radiusPixels = radiusPixels;
        rollDetection.angleDegrees = std::atan2(dy, dx) * 180.0 / kPi;
        rollDetection.brightness = brightness;
        rollDetection.reliability = reliability;
        rollDetection.shapeScore = shapeScore;
        rollDetection.score = brightGuidedDetectionPriorityScore(
            detection,
            brightness,
            reliability,
            shapeScore);
        rollConsensusDetections.append(rollDetection);
    }
    std::sort(rollConsensusDetections.begin(), rollConsensusDetections.end(), [](const RollConsensusDetection& lhs, const RollConsensusDetection& rhs) {
        if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
            return lhs.score > rhs.score;
        }
        return lhs.brightness > rhs.brightness;
    });
    const int rollConsensusDetectionLimit = std::min(
        static_cast<int>(rollConsensusDetections.size()),
        96);
    if (rollConsensusDetections.size() > rollConsensusDetectionLimit) {
        rollConsensusDetections.resize(rollConsensusDetectionLimit);
    }

    struct RollConsensusPair
    {
        int catalogAnchorIndex = -1;
        int detectionIndex = -1;
        double rollDegrees = 0.0;
        double radialErrorPixels = 0.0;
        double score = 0.0;
        double magnitude = 0.0;
    };
    QVector<RollConsensusPair> rollConsensusPairs;
    rollConsensusPairs.reserve(1024);
    qint64 rollConsensusPairCandidates = 0;
    const int rollConsensusCatalogLimit = std::min(
        static_cast<int>(anchorCatalogStars.size()),
        72);
    const double rollConsensusRadialTolerancePixels = std::min(
        maxImageDimension * 0.20,
        std::max(
            static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 5.0,
            std::max(38.0, maxImageDimension * 0.030)));
    for (int catalogAnchorIndex = 0; catalogAnchorIndex < rollConsensusCatalogLimit; ++catalogAnchorIndex)
    {
        const ProjectedAnchorCatalogStar& catalogAnchor = anchorCatalogStars[catalogAnchorIndex];
        const double catalogDx = catalogAnchor.projectedPoint.x() - principalPoint.x();
        const double catalogDy = principalPoint.y() - catalogAnchor.projectedPoint.y();
        const double catalogRadiusPixels = std::hypot(catalogDx, catalogDy);
        if (catalogRadiusPixels < 18.0) {
            continue;
        }

        const double catalogAngleDegrees = std::atan2(catalogDy, catalogDx) * 180.0 / kPi;
        const bool namedCatalogStar = (catalogAnchor.visibleStar.catalogIndex >= 0)
            && (catalogAnchor.visibleStar.catalogIndex < catalogContext.catalogStars.size())
            && isNamedSparseGuidedCatalogStar(catalogContext.catalogStars[catalogAnchor.visibleStar.catalogIndex]);
        QVector<RollConsensusPair> perCatalogPairs;
        perCatalogPairs.reserve(12);
        for (int detectionRank = 0; detectionRank < rollConsensusDetections.size(); ++detectionRank)
        {
            const RollConsensusDetection& detection = rollConsensusDetections[detectionRank];
            const CameraPipelineStarDetection& starDetection = starDetections[detection.detectionIndex];
            if (isImplausiblyCompactBrightCatalogDetection(
                    starDetection,
                    catalogAnchor.visibleStar.magnitude,
                    true))
            {
                continue;
            }

            const double radialErrorPixels = std::fabs(detection.radiusPixels - catalogRadiusPixels);
            if (radialErrorPixels > rollConsensusRadialTolerancePixels) {
                continue;
            }
            ++rollConsensusPairCandidates;

            const double detectionRankFraction = rollConsensusDetections.size() > 1
                ? static_cast<double>(detectionRank) / static_cast<double>(rollConsensusDetections.size() - 1)
                : 0.0;
            const double catalogRankFraction = rollConsensusCatalogLimit > 1
                ? static_cast<double>(catalogAnchorIndex) / static_cast<double>(rollConsensusCatalogLimit - 1)
                : 0.0;
            const double rankError = std::fabs(detectionRankFraction - catalogRankFraction);
            const double reliabilityBonus = std::min(
                65.0,
                std::log1p(std::max(0.0, detection.reliability)) * 7.0);
            const double shapeBonus = std::min(
                50.0,
                std::max(0.0, detection.shapeScore) * 0.65);
            const double brightnessBonus = std::min(
                30.0,
                std::log1p(std::max(0.0, detection.brightness)) * 2.2);
            RollConsensusPair pair;
            pair.catalogAnchorIndex = catalogAnchorIndex;
            pair.detectionIndex = detection.detectionIndex;
            pair.rollDegrees = normalizeDegrees(detection.angleDegrees - catalogAngleDegrees + 180.0) - 180.0;
            pair.radialErrorPixels = radialErrorPixels;
            pair.magnitude = catalogAnchor.visibleStar.magnitude;
            pair.score = radialErrorPixels * 0.45
                + catalogAnchor.visibleStar.magnitude * 8.0
                + rankError * 60.0
                - reliabilityBonus
                - shapeBonus
                - brightnessBonus
                - (namedCatalogStar ? 14.0 : 0.0);
            perCatalogPairs.append(pair);
        }

        std::sort(perCatalogPairs.begin(), perCatalogPairs.end(), [](const RollConsensusPair& lhs, const RollConsensusPair& rhs) {
            if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
                return lhs.score < rhs.score;
            }
            return lhs.radialErrorPixels < rhs.radialErrorPixels;
        });
        const int perCatalogPairLimit = catalogAnchor.visibleStar.magnitude <= 12.0 ? 8 : 5;
        if (perCatalogPairs.size() > perCatalogPairLimit) {
            perCatalogPairs.resize(perCatalogPairLimit);
        }
        rollConsensusPairs += perCatalogPairs;
    }
    std::sort(rollConsensusPairs.begin(), rollConsensusPairs.end(), [](const RollConsensusPair& lhs, const RollConsensusPair& rhs) {
        if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
            return lhs.score < rhs.score;
        }
        return lhs.magnitude < rhs.magnitude;
    });
    if (rollConsensusPairs.size() > 640) {
        rollConsensusPairs.resize(640);
    }

    qint64 rollConsensusDirectEvaluations = 0;
    qint64 rollConsensusDirectSeeds = 0;
    {
        QSet<int> triedRollBins;
        const int directRollEvaluationLimit = 160;
        const int directRollSeedLimit = 48;
        for (const RollConsensusPair& pair : rollConsensusPairs)
        {
            if (isCancellationRequested()
                || (rollConsensusDirectEvaluations >= directRollEvaluationLimit)
                || (rollConsensusDirectSeeds >= directRollSeedLimit))
            {
                break;
            }

            const int rollBin = static_cast<int>(std::floor((pair.rollDegrees + 180.0) * 2.0));
            if (triedRollBins.contains(rollBin)) {
                continue;
            }
            triedRollBins.insert(rollBin);

            Evaluation candidate = evaluatePose(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                settings.m_azimuth,
                settings.m_elevation,
                pair.rollDegrees,
                settings.m_fov,
                nullptr,
                fixedCenterOffsetX,
                fixedCenterOffsetY,
                fixedDistortionK1);
            ++rollConsensusDirectEvaluations;
            candidate.anchored = true;
            candidate.anchorDetectionIndex = pair.detectionIndex;
            candidate.anchorCatalogIndex = anchorCatalogStars[pair.catalogAnchorIndex].visibleStar.catalogIndex;
            const bool strongDirectAnchorPair =
                (pair.magnitude <= 12.0)
                && (pair.radialErrorPixels <= std::max(
                    static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 2.5,
                    rollConsensusRadialTolerancePixels * 0.50));
            const double directRmsCap = strongDirectAnchorPair
                ? std::min(
                    static_cast<double>(settings.m_plateSolveFinalMatchRadius),
                    24.0)
                : std::min(
                    static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.80,
                    20.0);
            if (!candidate.valid
                || (candidate.matchCount < std::max(4, settings.m_plateSolveMinMatches))
                || (candidate.rmsErrorPixels > directRmsCap)
                || (!strongDirectAnchorPair && !hasAcceptableBrightnessConsistency(candidate)))
            {
                continue;
            }

            seeds.append(candidate);
            ++rollConsensusDirectSeeds;
        }

        if (!plateSolveStartUsesRoll(settings) && isNarrowGuidedDirectionSolve(settings))
        {
            static const std::array<double, 17> localRollOffsets {{
                -20.0, -15.0, -12.5, -10.0, -7.5, -5.0, -2.5, 0.0, 2.5,
                5.0, 7.5, 10.0, 12.5, 15.0, 20.0, -30.0, 30.0
            }};
            QVector<int> localRollDetectionIndices;
            localRollDetectionIndices.reserve(starDetections.size());
            for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex) {
                localRollDetectionIndices.append(detectionIndex);
            }
            QSet<int> triedLocalRollBins;
            for (double rollOffset : localRollOffsets)
            {
                if (isCancellationRequested()
                    || (rollConsensusDirectEvaluations >= directRollEvaluationLimit)
                    || (rollConsensusDirectSeeds >= directRollSeedLimit))
                {
                    break;
                }

                const double rollDegrees = normalizeDegrees(settings.m_roll + rollOffset + 180.0) - 180.0;
                const int rollBin = static_cast<int>(std::floor((rollDegrees + 180.0) * 2.0));
                if (triedLocalRollBins.contains(rollBin)) {
                    continue;
                }
                triedLocalRollBins.insert(rollBin);

                Evaluation candidate = evaluatePose(
                    settings,
                    catalogContext,
                    imageSize,
                    captureDateTimeUtc,
                    starDetections,
                    localRollDetectionIndices,
                    settings.m_azimuth,
                    settings.m_elevation,
                    rollDegrees,
                    settings.m_fov,
                    nullptr,
                    fixedCenterOffsetX,
                    fixedCenterOffsetY,
                    fixedDistortionK1);
                ++rollConsensusDirectEvaluations;
                if (!candidate.valid
                    || (candidate.matchCount < std::max(4, settings.m_plateSolveMinMatches))
                    || (candidate.rmsErrorPixels > std::min(
                        static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.90,
                        22.0)))
                {
                    continue;
                }

                seeds.append(candidate);
                ++rollConsensusDirectSeeds;
            }
        }
    }

    struct RollConsensusTriangleCandidate
    {
        std::array<int, 3> pairIndices{{-1, -1, -1}};
        double score = 0.0;
    };
    QVector<RollConsensusTriangleCandidate> rollConsensusTriangleCandidates;
    rollConsensusTriangleCandidates.reserve(768);
    qint64 rollConsensusClusters = 0;
    qint64 rollConsensusSupportPairs = 0;
    qint64 rollConsensusTriangleCandidateCount = 0;
    const double rollConsensusToleranceDegrees = std::max(4.0, std::min(12.0, settings.m_fov * 5.0));
    const int rollConsensusClusterLimit = 96;
    for (int seedPairIndex = 0;
         (seedPairIndex < rollConsensusPairs.size()) && (rollConsensusClusters < rollConsensusClusterLimit);
         ++seedPairIndex)
    {
        if (isCancellationRequested()) {
            return seeds;
        }

        QVector<int> supportPairIndices;
        supportPairIndices.reserve(16);
        QSet<int> usedCatalogAnchors;
        QSet<int> usedDetections;
        double weightedSin = 0.0;
        double weightedCos = 0.0;
        for (int pairIndex = seedPairIndex; pairIndex < rollConsensusPairs.size(); ++pairIndex)
        {
            const RollConsensusPair& pair = rollConsensusPairs[pairIndex];
            const double rollDelta = std::fabs(normalizeDegrees(pair.rollDegrees - rollConsensusPairs[seedPairIndex].rollDegrees + 180.0) - 180.0);
            if (rollDelta > rollConsensusToleranceDegrees) {
                continue;
            }
            if (usedCatalogAnchors.contains(pair.catalogAnchorIndex)
                || usedDetections.contains(pair.detectionIndex))
            {
                continue;
            }

            supportPairIndices.append(pairIndex);
            usedCatalogAnchors.insert(pair.catalogAnchorIndex);
            usedDetections.insert(pair.detectionIndex);
            const double weight = 1.0 / (1.0 + std::max(0.0, pair.score));
            weightedSin += std::sin(degToRad(pair.rollDegrees)) * weight;
            weightedCos += std::cos(degToRad(pair.rollDegrees)) * weight;
            if (supportPairIndices.size() >= 10) {
                break;
            }
        }

        if (supportPairIndices.size() < 3) {
            continue;
        }
        ++rollConsensusClusters;
        rollConsensusSupportPairs += supportPairIndices.size();

        const double clusterRollDegrees = std::atan2(weightedSin, weightedCos) * 180.0 / kPi;
        for (int a = 0; a < supportPairIndices.size(); ++a)
        {
            for (int b = a + 1; b < supportPairIndices.size(); ++b)
            {
                for (int c = b + 1; c < supportPairIndices.size(); ++c)
                {
                    const RollConsensusPair& pairA = rollConsensusPairs[supportPairIndices[a]];
                    const RollConsensusPair& pairB = rollConsensusPairs[supportPairIndices[b]];
                    const RollConsensusPair& pairC = rollConsensusPairs[supportPairIndices[c]];
                    const double rollSpread =
                        std::fabs(normalizeDegrees(pairA.rollDegrees - clusterRollDegrees + 180.0) - 180.0)
                        + std::fabs(normalizeDegrees(pairB.rollDegrees - clusterRollDegrees + 180.0) - 180.0)
                        + std::fabs(normalizeDegrees(pairC.rollDegrees - clusterRollDegrees + 180.0) - 180.0);
                    ++rollConsensusTriangleCandidateCount;
                    RollConsensusTriangleCandidate candidate;
                    candidate.pairIndices = {{supportPairIndices[a], supportPairIndices[b], supportPairIndices[c]}};
                    candidate.score = pairA.score + pairB.score + pairC.score + rollSpread * 10.0;
                    rollConsensusTriangleCandidates.append(candidate);
                }
            }
        }
    }

    std::sort(rollConsensusTriangleCandidates.begin(), rollConsensusTriangleCandidates.end(), [](const RollConsensusTriangleCandidate& lhs, const RollConsensusTriangleCandidate& rhs) {
        if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
            return lhs.score < rhs.score;
        }
        return lhs.pairIndices[0] < rhs.pairIndices[0];
    });
    if (rollConsensusTriangleCandidates.size() > 768) {
        rollConsensusTriangleCandidates.resize(768);
    }

    qint64 rollConsensusSeedEvaluations = 0;
    qint64 rollConsensusSeedValid = 0;
    qint64 rollConsensusAnchorSupported = 0;
    qint64 rollConsensusSeeds = 0;
    qint64 rollConsensusVerifiedSeeds = 0;
    int rollConsensusMaxAnchors = 0;
    const int rollConsensusEvaluationLimit = 1200;
    const int rollConsensusSeedLimit = 96;
    const std::array<double, 3> rollConsensusFovScales = plateSolveStartUsesFov(settings)
        ? std::array<double, 3>{{1.0, 1.0, 1.0}}
        : std::array<double, 3>{{0.98, 1.0, 1.02}};
    for (const RollConsensusTriangleCandidate& rollTriangleCandidate : rollConsensusTriangleCandidates)
    {
        if (isCancellationRequested()
            || (rollConsensusSeedEvaluations >= rollConsensusEvaluationLimit)
            || (seeds.size() >= rollConsensusSeedLimit))
        {
            break;
        }

        const RollConsensusPair& pairA = rollConsensusPairs[rollTriangleCandidate.pairIndices[0]];
        const RollConsensusPair& pairB = rollConsensusPairs[rollTriangleCandidate.pairIndices[1]];
        const RollConsensusPair& pairC = rollConsensusPairs[rollTriangleCandidate.pairIndices[2]];
        const std::array<int, 3> anchorDetectionIndices {{
            pairA.detectionIndex,
            pairB.detectionIndex,
            pairC.detectionIndex
        }};
        const std::array<int, 3> anchorCatalogIndices {{
            anchorCatalogStars[pairA.catalogAnchorIndex].visibleStar.catalogIndex,
            anchorCatalogStars[pairB.catalogAnchorIndex].visibleStar.catalogIndex,
            anchorCatalogStars[pairC.catalogAnchorIndex].visibleStar.catalogIndex
        }};
        QVector<int> allowedCatalogIndices {
            anchorCatalogIndices[0],
            anchorCatalogIndices[1],
            anchorCatalogIndices[2]
        };

        double previousSeedFov = -1.0;
        for (double fovScale : rollConsensusFovScales)
        {
            if (isCancellationRequested()
                || (rollConsensusSeedEvaluations >= rollConsensusEvaluationLimit)
                || (seeds.size() >= rollConsensusSeedLimit))
            {
                break;
            }

            const double seedFov = std::clamp(
                static_cast<double>(settings.m_fov) * fovScale,
                static_cast<double>(CameraSettings::m_minFov),
                static_cast<double>(CameraSettings::m_maxFov));
            if (std::fabs(seedFov - previousSeedFov) < 1e-9) {
                continue;
            }
            previousSeedFov = seedFov;
            if (!seedFovCompatibleWithStartFov(settings, seedFov)) {
                continue;
            }

            const SkyProjector baseProjector = createProjector(
                settings,
                imageSize,
                0.0,
                90.0,
                0.0,
                seedFov,
                fixedCenterOffsetX,
                fixedCenterOffsetY,
                fixedDistortionK1);
            if (!baseProjector.valid) {
                continue;
            }

            std::array<SkyVector, 3> sourceVectors;
            bool haveSourceVectors = true;
            for (int idx = 0; idx < 3; ++idx)
            {
                if (!unprojectPixelToVector(
                        baseProjector,
                        starDetections[anchorDetectionIndices[idx]].m_center,
                        sourceVectors[idx]))
                {
                    haveSourceVectors = false;
                    break;
                }
            }
            if (!haveSourceVectors) {
                continue;
            }

            const std::array<SkyVector, 3> targetVectors {{
                anchorCatalogStars[pairA.catalogAnchorIndex].visibleStar.vector,
                anchorCatalogStars[pairB.catalogAnchorIndex].visibleStar.vector,
                anchorCatalogStars[pairC.catalogAnchorIndex].visibleStar.vector
            }};
            double seedAzimuth = 0.0;
            double seedElevation = 0.0;
            double seedRoll = 0.0;
            double triangleRmsAngularError = std::numeric_limits<double>::infinity();
            double triangleMaxAngularError = std::numeric_limits<double>::infinity();
            if (!poseFromThreeVectorPairs(
                    baseProjector,
                    sourceVectors,
                    targetVectors,
                    seedAzimuth,
                    seedElevation,
                    seedRoll,
                    &triangleRmsAngularError,
                    &triangleMaxAngularError))
            {
                continue;
            }
            Q_UNUSED(triangleRmsAngularError)
            Q_UNUSED(triangleMaxAngularError)

            const double directionDelta = std::acos(std::clamp(
                dot(vectorFromAltAz(seedAzimuth, seedElevation), vectorFromAltAz(settings.m_azimuth, settings.m_elevation)),
                -1.0,
                1.0)) * 180.0 / kPi;
            if (directionDelta > startDirectionMaxDelta) {
                continue;
            }

            Evaluation seededCandidate = evaluatePose(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                seedAzimuth,
                seedElevation,
                seedRoll,
                seedFov,
                &allowedCatalogIndices,
                fixedCenterOffsetX,
                fixedCenterOffsetY,
                fixedDistortionK1);
            ++rollConsensusSeedEvaluations;
            if (!seededCandidate.valid) {
                continue;
            }
            ++rollConsensusSeedValid;

            const double anchorDistancePixels = std::min(
                96.0,
                std::max(
                    static_cast<double>(settings.m_plateSolveMatchRadius),
                    static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 3.0));
            const int anchorSupport = countProjectedAnchorSupport(
                settings,
                catalogContext,
                imageSize,
                starDetections,
                seededCandidate,
                anchorDetectionIndices,
                anchorCatalogIndices,
                anchorDistancePixels);
            rollConsensusMaxAnchors = std::max(rollConsensusMaxAnchors, anchorSupport);
            if (anchorSupport < 3) {
                continue;
            }
            ++rollConsensusAnchorSupported;
            seededCandidate.anchored = true;
            seededCandidate.guidedTriangle = true;
            seededCandidate.anchorDetectionIndex = anchorDetectionIndices[0];
            seededCandidate.anchorCatalogIndex = anchorCatalogIndices[0];
            seededCandidate.secondaryAnchorDetectionIndex = anchorDetectionIndices[1];
            seededCandidate.secondaryAnchorCatalogIndex = anchorCatalogIndices[1];
            seededCandidate.tertiaryAnchorDetectionIndex = anchorDetectionIndices[2];
            seededCandidate.tertiaryAnchorCatalogIndex = anchorCatalogIndices[2];
            seededCandidate.matches = {
                { anchorDetectionIndices[0], anchorCatalogIndices[0], 0.0 },
                { anchorDetectionIndices[1], anchorCatalogIndices[1], 0.0 },
                { anchorDetectionIndices[2], anchorCatalogIndices[2], 0.0 }
            };
            seededCandidate.matchCount = seededCandidate.matches.size();
            seededCandidate.rmsErrorPixels = std::min(
                seededCandidate.rmsErrorPixels,
                static_cast<double>(settings.m_plateSolveFinalMatchRadius));
            seeds.append(seededCandidate);
            ++rollConsensusSeeds;

            Evaluation candidate = evaluatePose(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                seededCandidate.azimuthDegrees,
                seededCandidate.elevationDegrees,
                seededCandidate.rollDegrees,
                seededCandidate.fovDegrees,
                nullptr,
                fixedCenterOffsetX,
                fixedCenterOffsetY,
                fixedDistortionK1);
            candidate.anchored = true;
            candidate.guidedTriangle = true;
            candidate.anchorDetectionIndex = anchorDetectionIndices[0];
            candidate.anchorCatalogIndex = anchorCatalogIndices[0];
            candidate.secondaryAnchorDetectionIndex = anchorDetectionIndices[1];
            candidate.secondaryAnchorCatalogIndex = anchorCatalogIndices[1];
            candidate.tertiaryAnchorDetectionIndex = anchorDetectionIndices[2];
            candidate.tertiaryAnchorCatalogIndex = anchorCatalogIndices[2];

            const double guidedTriangleSeedRmsCap = std::min(
                std::max(18.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.75),
                static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.90);
            if (candidate.valid
                && (candidate.matchCount >= std::max(4, settings.m_plateSolveMinMatches))
                && (candidate.rmsErrorPixels <= guidedTriangleSeedRmsCap)
                && hasAcceptableBrightnessConsistency(candidate))
            {
                seeds.append(candidate);
                ++rollConsensusSeeds;
            }

            Evaluation verifiedCandidate = verifyBlindSeedCandidate(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                candidate);
            if (verifiedCandidate.valid)
            {
                verifiedCandidate.anchored = true;
                verifiedCandidate.guidedTriangle = true;
                verifiedCandidate.anchorDetectionIndex = anchorDetectionIndices[0];
                verifiedCandidate.anchorCatalogIndex = anchorCatalogIndices[0];
                verifiedCandidate.secondaryAnchorDetectionIndex = anchorDetectionIndices[1];
                verifiedCandidate.secondaryAnchorCatalogIndex = anchorCatalogIndices[1];
                verifiedCandidate.tertiaryAnchorDetectionIndex = anchorDetectionIndices[2];
                verifiedCandidate.tertiaryAnchorCatalogIndex = anchorCatalogIndices[2];
                seeds.append(verifiedCandidate);
                ++rollConsensusSeeds;
                ++rollConsensusVerifiedSeeds;
            }
        }
    }
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusDetections"), rollConsensusDetections.size());
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusPairs"), rollConsensusPairs.size());
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusPairCandidates"), rollConsensusPairCandidates);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusDirectEvaluations"), rollConsensusDirectEvaluations);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusDirectSeeds"), rollConsensusDirectSeeds);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusClusters"), rollConsensusClusters);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusSupportPairs"), rollConsensusSupportPairs);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusTriangleCandidates"), rollConsensusTriangleCandidateCount);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusKept"), rollConsensusTriangleCandidates.size());
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusEvaluations"), rollConsensusSeedEvaluations);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusSeedValid"), rollConsensusSeedValid);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusAnchorSupported"), rollConsensusAnchorSupported);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusMaxAnchors"), rollConsensusMaxAnchors);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusSeeds"), rollConsensusSeeds);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusVerifiedSeeds"), rollConsensusVerifiedSeeds);
#else
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusDetections"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusPairs"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusPairCandidates"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusDirectEvaluations"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusDirectSeeds"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusClusters"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusSupportPairs"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusTriangleCandidates"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusKept"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusEvaluations"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusSeedValid"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusAnchorSupported"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusMaxAnchors"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusSeeds"), 0);
    recordProfileMetric(QStringLiteral("search.guidedRollConsensusVerifiedSeeds"), 0);
#endif

    struct ProjectedAnchorCandidate
    {
        int catalogAnchorIndex = -1;
        int detectionIndex = -1;
        double score = 0.0;
        double distancePixels = 0.0;
    };
    const bool expandedProjectedAnchorSearch = settings.m_plateSolveMaxMagnitude <= 14.0;
    const int projectedCatalogLimit = std::min(
        expandedProjectedAnchorSearch ? 48 : 32,
        static_cast<int>(anchorCatalogStars.size()));
    const double projectedAnchorTolerancePixels = std::min(
        maxImageDimension * 0.32,
        std::max(
            static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 10.0,
            std::max(180.0, maxImageDimension * 0.16)));
    QVector<QVector<ProjectedAnchorCandidate>> projectedAnchorCandidates(projectedCatalogLimit);
    qint64 projectedAnchorPairs = 0;
    for (int catalogAnchorIndex = 0; catalogAnchorIndex < projectedCatalogLimit; ++catalogAnchorIndex)
    {
        const ProjectedAnchorCatalogStar& catalogAnchor = anchorCatalogStars[catalogAnchorIndex];
        QVector<ProjectedAnchorCandidate>& perCatalogAnchors = projectedAnchorCandidates[catalogAnchorIndex];
        perCatalogAnchors.reserve(12);
        const bool namedCatalogStar = (catalogAnchor.visibleStar.catalogIndex >= 0)
            && (catalogAnchor.visibleStar.catalogIndex < catalogContext.catalogStars.size())
            && isNamedSparseGuidedCatalogStar(catalogContext.catalogStars[catalogAnchor.visibleStar.catalogIndex]);
        for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex)
        {
            const CameraPipelineStarDetection& detection = starDetections[detectionIndex];
            if (!isDetectionUsableForBrightPrior(detection)) {
                continue;
            }
            if (isImplausiblyCompactBrightCatalogDetection(
                    detection,
                    catalogAnchor.visibleStar.magnitude,
                    true))
            {
                continue;
            }

            const double distancePixels = pointDistancePixels(detection.m_center, catalogAnchor.projectedPoint);
            if (distancePixels > projectedAnchorTolerancePixels) {
                continue;
            }

            const double detectionRank = ((detectionIndex >= 0) && (detectionIndex < detectionRanks.size()))
                ? detectionRanks[detectionIndex]
                : 0.5;
            const double reliability = cachedDetectionReliabilityMetric(starDetections, detectionIndex);
            const double shapeScore = narrowGuidedAnchorShapeScore(detection);
            const double brightness = cachedDetectionBrightnessMetric(starDetections, detectionIndex);
            const double score = distancePixels * (expandedProjectedAnchorSearch ? 0.40 : 0.45)
                + catalogAnchor.visibleStar.magnitude * (expandedProjectedAnchorSearch ? 6.0 : 9.0)
                + detectionRank * 55.0
                - std::min(70.0, std::log1p(std::max(0.0, reliability)) * 8.0)
                - std::min(55.0, shapeScore * 0.9)
                - std::min(30.0, std::log1p(std::max(0.0, brightness)) * 2.0)
                - (namedCatalogStar ? (expandedProjectedAnchorSearch ? 30.0 : 12.0) : 0.0);
            perCatalogAnchors.append({
                catalogAnchorIndex,
                detectionIndex,
                score,
                distancePixels
            });
        }

        std::sort(perCatalogAnchors.begin(), perCatalogAnchors.end(), [](const ProjectedAnchorCandidate& lhs, const ProjectedAnchorCandidate& rhs) {
            if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
                return lhs.score < rhs.score;
            }
            return lhs.distancePixels < rhs.distancePixels;
        });
        const int perCatalogLimit = catalogAnchor.visibleStar.magnitude <= 12.0
            ? (expandedProjectedAnchorSearch ? 12 : 8)
            : (expandedProjectedAnchorSearch ? 6 : 5);
        if (perCatalogAnchors.size() > perCatalogLimit) {
            perCatalogAnchors.resize(perCatalogLimit);
        }
        projectedAnchorPairs += perCatalogAnchors.size();
    }

    struct ProjectedTriangleCandidate
    {
        std::array<int, 3> catalogAnchorIndices{{-1, -1, -1}};
        std::array<int, 3> detectionIndices{{-1, -1, -1}};
        double score = 0.0;
        double seedFov = 0.0;
        double similarityRmsPixels = 0.0;
        double brightnessMagnitudeError = 0.0;
    };
    QVector<ProjectedTriangleCandidate> projectedTriangleCandidates;
    const int projectedTriangleKeepLimit = expandedProjectedAnchorSearch ? 1024 : 768;
    projectedTriangleCandidates.reserve(projectedTriangleKeepLimit);
    qint64 projectedTriangleCandidateCount = 0;
    qint64 projectedTriangleRatioMatches = 0;
    const double projectedTriangleRatioTolerance = 0.10;
    const double projectedTriangleSimilarityTolerancePixels = std::max(
        static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 4.0,
        std::min(160.0, projectedAnchorTolerancePixels * 0.55));

    for (int i = 0; i < projectedCatalogLimit; ++i)
    {
        if (projectedAnchorCandidates[i].isEmpty()) {
            continue;
        }
        for (int j = i + 1; j < projectedCatalogLimit; ++j)
        {
            if (projectedAnchorCandidates[j].isEmpty()) {
                continue;
            }
            for (int k = j + 1; k < projectedCatalogLimit; ++k)
            {
                if (isCancellationRequested()) {
                    return seeds;
                }
                if (projectedAnchorCandidates[k].isEmpty()) {
                    continue;
                }

                const std::array<QPointF, 3> catalogProjectedPoints {{
                    anchorCatalogStars[i].projectedPoint,
                    anchorCatalogStars[j].projectedPoint,
                    anchorCatalogStars[k].projectedPoint
                }};
                const TriangleSignature catalogSignature = buildTriangleSignature(catalogProjectedPoints);
                if (catalogSignature.longestDistance < 20.0) {
                    continue;
                }
                const std::array<VisibleCatalogStar, 3> projectedStars {{
                    anchorCatalogStars[i].visibleStar,
                    anchorCatalogStars[j].visibleStar,
                    anchorCatalogStars[k].visibleStar
                }};

                for (const ProjectedAnchorCandidate& anchorA : projectedAnchorCandidates[i])
                {
                    for (const ProjectedAnchorCandidate& anchorB : projectedAnchorCandidates[j])
                    {
                        if (anchorA.detectionIndex == anchorB.detectionIndex) {
                            continue;
                        }
                        for (const ProjectedAnchorCandidate& anchorC : projectedAnchorCandidates[k])
                        {
                            if ((anchorA.detectionIndex == anchorC.detectionIndex)
                                || (anchorB.detectionIndex == anchorC.detectionIndex))
                            {
                                continue;
                            }
                            ++projectedTriangleCandidateCount;

                            const std::array<int, 3> anchorDetectionIndices {{
                                anchorA.detectionIndex,
                                anchorB.detectionIndex,
                                anchorC.detectionIndex
                            }};
                            if (!triangleBrightnessOrderCompatible(
                                    starDetections,
                                    anchorDetectionIndices,
                                    projectedStars,
                                    std::array<int, 3>{{0, 1, 2}}))
                            {
                                continue;
                            }
                            const double brightnessMagnitudeError = triangleBrightnessMagnitudeError(
                                starDetections,
                                anchorDetectionIndices,
                                projectedStars,
                                std::array<int, 3>{{0, 1, 2}});
                            if (!std::isfinite(brightnessMagnitudeError)
                                || (brightnessMagnitudeError > 4.0))
                            {
                                continue;
                            }

                            const std::array<QPointF, 3> detectionPoints {{
                                starDetections[anchorA.detectionIndex].m_center,
                                starDetections[anchorB.detectionIndex].m_center,
                                starDetections[anchorC.detectionIndex].m_center
                            }};
                            const TriangleSignature detectionSignature = buildTriangleSignature(detectionPoints);
                            if (detectionSignature.longestDistance < 20.0) {
                                continue;
                            }

                            const double ratioError =
                                std::fabs(detectionSignature.ratioShortToLong - catalogSignature.ratioShortToLong)
                                + std::fabs(detectionSignature.ratioMidToLong - catalogSignature.ratioMidToLong);
                            if (ratioError > projectedTriangleRatioTolerance) {
                                continue;
                            }
                            ++projectedTriangleRatioMatches;

                            double similarityScale = 1.0;
                            double similarityRotationDegrees = 0.0;
                            double similarityRmsPixels = std::numeric_limits<double>::infinity();
                            if (!estimateScreenSimilarity(
                                    catalogProjectedPoints,
                                    detectionPoints,
                                    similarityScale,
                                    similarityRotationDegrees,
                                    similarityRmsPixels)
                                || (similarityRmsPixels > projectedTriangleSimilarityTolerancePixels))
                            {
                                continue;
                            }
                            Q_UNUSED(similarityRotationDegrees)

                            const double seedFov = plateSolveStartUsesFov(settings)
                                ? std::clamp(
                                    static_cast<double>(settings.m_fov),
                                    static_cast<double>(CameraSettings::m_minFov),
                                    static_cast<double>(CameraSettings::m_maxFov))
                                : std::clamp(
                                    static_cast<double>(settings.m_fov) / std::max(0.50, std::min(2.0, similarityScale)),
                                    static_cast<double>(CameraSettings::m_minFov),
                                    static_cast<double>(CameraSettings::m_maxFov));
                            if (!seedFovCompatibleWithStartFov(settings, seedFov)) {
                                continue;
                            }

                            const double distanceScore =
                                anchorA.distancePixels + anchorB.distancePixels + anchorC.distancePixels;
                            const double anchorScore = anchorA.score + anchorB.score + anchorC.score;
                            const double magnitudeSum =
                                projectedStars[0].magnitude + projectedStars[1].magnitude + projectedStars[2].magnitude;
                            ProjectedTriangleCandidate candidate;
                            candidate.catalogAnchorIndices = {{i, j, k}};
                            candidate.detectionIndices = anchorDetectionIndices;
                            candidate.seedFov = seedFov;
                            candidate.similarityRmsPixels = similarityRmsPixels;
                            candidate.brightnessMagnitudeError = brightnessMagnitudeError;
                            candidate.score = anchorScore
                                + similarityRmsPixels * 4.0
                                + ratioError * 180.0
                                + distanceScore * 0.10
                                + magnitudeSum * 2.5
                                + brightnessMagnitudeError * 45.0;
                            projectedTriangleCandidates.append(candidate);
                        }
                    }
                }
            }
        }
    }

    std::sort(projectedTriangleCandidates.begin(), projectedTriangleCandidates.end(), [](const ProjectedTriangleCandidate& lhs, const ProjectedTriangleCandidate& rhs) {
        if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
            return lhs.score < rhs.score;
        }
        if (!qFuzzyCompare(lhs.brightnessMagnitudeError + 1.0, rhs.brightnessMagnitudeError + 1.0)) {
            return lhs.brightnessMagnitudeError < rhs.brightnessMagnitudeError;
        }
        return lhs.similarityRmsPixels < rhs.similarityRmsPixels;
    });
    if (projectedTriangleCandidates.size() > projectedTriangleKeepLimit) {
        projectedTriangleCandidates.resize(projectedTriangleKeepLimit);
    }

    qint64 projectedTriangleEvaluations = 0;
    qint64 projectedTriangleSeedValid = 0;
    qint64 projectedTriangleAnchorSupported = 0;
    qint64 projectedTriangleSeeds = 0;
    qint64 projectedTriangleVerifiedSeeds = 0;
    int projectedTriangleMaxAnchors = 0;
    const int projectedTriangleEvaluationLimit = expandedProjectedAnchorSearch ? 1600 : 1200;
    for (const ProjectedTriangleCandidate& projectedTriangleCandidate : projectedTriangleCandidates)
    {
        if (isCancellationRequested()
            || (projectedTriangleEvaluations >= projectedTriangleEvaluationLimit)
            || (seeds.size() >= 96))
        {
            break;
        }

        const std::array<VisibleCatalogStar, 3> projectedStars {{
            anchorCatalogStars[projectedTriangleCandidate.catalogAnchorIndices[0]].visibleStar,
            anchorCatalogStars[projectedTriangleCandidate.catalogAnchorIndices[1]].visibleStar,
            anchorCatalogStars[projectedTriangleCandidate.catalogAnchorIndices[2]].visibleStar
        }};
        const std::array<QPointF, 3> detectionPoints {{
            starDetections[projectedTriangleCandidate.detectionIndices[0]].m_center,
            starDetections[projectedTriangleCandidate.detectionIndices[1]].m_center,
            starDetections[projectedTriangleCandidate.detectionIndices[2]].m_center
        }};
        QVector<GuidedTrianglePoseSeed> poseSeeds = guidedTriangleSimilarityPoseSeeds(
            settings,
            imageSize,
            detectionPoints,
            projectedStars,
            std::array<int, 3>{{0, 1, 2}},
            settings.m_azimuth,
            settings.m_elevation,
            projectedTriangleCandidate.seedFov,
            fixedCenterOffsetX,
            fixedCenterOffsetY,
            fixedDistortionK1);

        const SkyProjector sourceProjector = createProjector(
            settings,
            imageSize,
            0.0,
            90.0,
            0.0,
            projectedTriangleCandidate.seedFov,
            fixedCenterOffsetX,
            fixedCenterOffsetY,
            fixedDistortionK1);
        if (sourceProjector.valid)
        {
            std::array<SkyVector, 3> sourceVectors;
            bool haveSourceVectors = true;
            for (int idx = 0; idx < 3; ++idx)
            {
                if (!unprojectPixelToVector(
                        sourceProjector,
                        detectionPoints[idx],
                        sourceVectors[idx]))
                {
                    haveSourceVectors = false;
                    break;
                }
            }
            if (haveSourceVectors)
            {
                const std::array<SkyVector, 3> targetVectors {{
                    projectedStars[0].vector,
                    projectedStars[1].vector,
                    projectedStars[2].vector
                }};
                GuidedTrianglePoseSeed wahbaSeed;
                wahbaSeed.fovDegrees = projectedTriangleCandidate.seedFov;
                double triangleRmsAngularError = std::numeric_limits<double>::infinity();
                double triangleMaxAngularError = std::numeric_limits<double>::infinity();
                if (poseFromThreeVectorPairs(
                        sourceProjector,
                        sourceVectors,
                        targetVectors,
                        wahbaSeed.azimuthDegrees,
                        wahbaSeed.elevationDegrees,
                        wahbaSeed.rollDegrees,
                        &triangleRmsAngularError,
                        &triangleMaxAngularError))
                {
                    Q_UNUSED(triangleRmsAngularError)
                    Q_UNUSED(triangleMaxAngularError)
                    poseSeeds.append(wahbaSeed);
                }
            }
        }

        const std::array<int, 3> anchorCatalogIndices {{
            projectedStars[0].catalogIndex,
            projectedStars[1].catalogIndex,
            projectedStars[2].catalogIndex
        }};
        const bool debugTargetTriangle =
            debugTriangleContainsCatalogStar(catalogContext, anchorCatalogIndices);
        QVector<int> allowedCatalogIndices {
            anchorCatalogIndices[0],
            anchorCatalogIndices[1],
            anchorCatalogIndices[2]
        };
        double previousAzimuth = std::numeric_limits<double>::quiet_NaN();
        double previousElevation = std::numeric_limits<double>::quiet_NaN();
        double previousRoll = std::numeric_limits<double>::quiet_NaN();
        for (const GuidedTrianglePoseSeed& poseSeed : poseSeeds)
        {
            if (isCancellationRequested()
                || (projectedTriangleEvaluations >= projectedTriangleEvaluationLimit)
                || (seeds.size() >= 96))
            {
                break;
            }
            if (std::isfinite(previousAzimuth)
                && (angularDistanceDegrees(previousAzimuth, poseSeed.azimuthDegrees) < 0.02)
                && (std::fabs(previousElevation - poseSeed.elevationDegrees) < 0.02)
                && (angularDistanceDegrees(previousRoll, poseSeed.rollDegrees) < 0.5))
            {
                continue;
            }
            previousAzimuth = poseSeed.azimuthDegrees;
            previousElevation = poseSeed.elevationDegrees;
            previousRoll = poseSeed.rollDegrees;

            const double directionDelta = std::acos(std::clamp(
                dot(vectorFromAltAz(poseSeed.azimuthDegrees, poseSeed.elevationDegrees), vectorFromAltAz(settings.m_azimuth, settings.m_elevation)),
                -1.0,
                1.0)) * 180.0 / kPi;
            if (directionDelta > startDirectionMaxDelta) {
                continue;
            }

            Evaluation seededCandidate = evaluatePose(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                poseSeed.azimuthDegrees,
                poseSeed.elevationDegrees,
                poseSeed.rollDegrees,
                poseSeed.fovDegrees,
                &allowedCatalogIndices,
                fixedCenterOffsetX,
                fixedCenterOffsetY,
                fixedDistortionK1);
            ++projectedTriangleEvaluations;
            if (!seededCandidate.valid) {
                if (debugTargetTriangle)
                {
                    qCDebug(cameraPlateSolverLog).noquote() << "CameraPlateSolver: projected triangle target seed invalid"
                        << debugTriangleAnchorSummary(
                            catalogContext,
                            starDetections,
                            projectedTriangleCandidate.detectionIndices,
                            anchorCatalogIndices);
                }
                continue;
            }
            ++projectedTriangleSeedValid;

            const double anchorDistancePixels = std::min(
                static_cast<double>(settings.m_plateSolveMatchRadius),
                std::max(6.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius)));
            const int anchorSupport = countProjectedAnchorSupport(
                settings,
                catalogContext,
                imageSize,
                starDetections,
                seededCandidate,
                projectedTriangleCandidate.detectionIndices,
                anchorCatalogIndices,
                anchorDistancePixels);
            projectedTriangleMaxAnchors = std::max(projectedTriangleMaxAnchors, anchorSupport);
            if (debugTargetTriangle)
            {
                qCDebug(cameraPlateSolverLog).noquote() << "CameraPlateSolver: projected triangle target seed"
                    << "anchorSupport" << anchorSupport
                    << "matches" << seededCandidate.matchCount
                    << "rms" << seededCandidate.rmsErrorPixels
                    << "Az" << seededCandidate.azimuthDegrees
                    << "El" << seededCandidate.elevationDegrees
                    << "Roll" << seededCandidate.rollDegrees
                    << "FoV" << seededCandidate.fovDegrees
                    << debugTriangleAnchorSummary(
                        catalogContext,
                        starDetections,
                        projectedTriangleCandidate.detectionIndices,
                        anchorCatalogIndices);
            }
            if (anchorSupport < 3) {
                continue;
            }
            ++projectedTriangleAnchorSupported;

            Evaluation candidate = evaluatePose(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                seededCandidate.azimuthDegrees,
                seededCandidate.elevationDegrees,
                seededCandidate.rollDegrees,
                seededCandidate.fovDegrees,
                nullptr,
                fixedCenterOffsetX,
                fixedCenterOffsetY,
                fixedDistortionK1);
            candidate.anchored = true;
            candidate.guidedTriangle = true;
            candidate.anchorDetectionIndex = projectedTriangleCandidate.detectionIndices[0];
            candidate.anchorCatalogIndex = anchorCatalogIndices[0];
            candidate.secondaryAnchorDetectionIndex = projectedTriangleCandidate.detectionIndices[1];
            candidate.secondaryAnchorCatalogIndex = anchorCatalogIndices[1];
            candidate.tertiaryAnchorDetectionIndex = projectedTriangleCandidate.detectionIndices[2];
            candidate.tertiaryAnchorCatalogIndex = anchorCatalogIndices[2];

            const double guidedTriangleSeedRmsCap = std::min(
                std::max(18.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.75),
                static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.90);
            if (candidate.valid
                && (candidate.matchCount >= std::max(4, settings.m_plateSolveMinMatches))
                && (candidate.rmsErrorPixels <= guidedTriangleSeedRmsCap)
                && hasAcceptableBrightnessConsistency(candidate))
            {
                seeds.append(candidate);
                ++projectedTriangleSeeds;
                if (debugTargetTriangle) {
                    qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: projected triangle target appended";
                }
            }

            Evaluation verifiedCandidate = verifyBlindSeedCandidate(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                candidate);
            if (verifiedCandidate.valid)
            {
                verifiedCandidate.anchored = true;
                verifiedCandidate.guidedTriangle = true;
                verifiedCandidate.anchorDetectionIndex = projectedTriangleCandidate.detectionIndices[0];
                verifiedCandidate.anchorCatalogIndex = anchorCatalogIndices[0];
                verifiedCandidate.secondaryAnchorDetectionIndex = projectedTriangleCandidate.detectionIndices[1];
                verifiedCandidate.secondaryAnchorCatalogIndex = anchorCatalogIndices[1];
                verifiedCandidate.tertiaryAnchorDetectionIndex = projectedTriangleCandidate.detectionIndices[2];
                verifiedCandidate.tertiaryAnchorCatalogIndex = anchorCatalogIndices[2];
                seeds.append(verifiedCandidate);
                ++projectedTriangleSeeds;
                ++projectedTriangleVerifiedSeeds;
                if (debugTargetTriangle) {
                    qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: projected triangle target verified appended";
                }
            }
        }
    }
    recordProfileMetric(QStringLiteral("search.guidedProjectedTriangleAnchorPairs"), projectedAnchorPairs);
    recordProfileMetric(QStringLiteral("search.guidedProjectedTriangleCandidates"), projectedTriangleCandidateCount);
    recordProfileMetric(QStringLiteral("search.guidedProjectedTriangleRatioMatches"), projectedTriangleRatioMatches);
    recordProfileMetric(QStringLiteral("search.guidedProjectedTriangleKept"), projectedTriangleCandidates.size());
    recordProfileMetric(QStringLiteral("search.guidedProjectedTriangleEvaluations"), projectedTriangleEvaluations);
    recordProfileMetric(QStringLiteral("search.guidedProjectedTriangleSeedValid"), projectedTriangleSeedValid);
    recordProfileMetric(QStringLiteral("search.guidedProjectedTriangleAnchorSupported"), projectedTriangleAnchorSupported);
    recordProfileMetric(QStringLiteral("search.guidedProjectedTriangleMaxAnchors"), projectedTriangleMaxAnchors);
    recordProfileMetric(QStringLiteral("search.guidedProjectedTriangleSeeds"), projectedTriangleSeeds);
    recordProfileMetric(QStringLiteral("search.guidedProjectedTriangleVerifiedSeeds"), projectedTriangleVerifiedSeeds);

    struct OrderedDetection
    {
        int detectionIndex = -1;
        double brightness = 0.0;
        double reliability = 0.0;
        double shapeScore = 0.0;
        double score = 0.0;
    };
    QVector<OrderedDetection> orderedDetections;
    orderedDetections.reserve(starDetections.size());
    for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex)
    {
        const CameraPipelineStarDetection& detection = starDetections[detectionIndex];
        if (!isDetectionUsableForBrightPrior(detection)) {
            continue;
        }

        const double brightness = cachedDetectionBrightnessMetric(starDetections, detectionIndex);
        const double reliability = cachedDetectionReliabilityMetric(starDetections, detectionIndex);
        const double shapeScore = narrowGuidedAnchorShapeScore(detection);
        if (!detection.m_saturated
            && (brightness < 12.0)
            && (reliability < 10.0)
            && (shapeScore < 0.0))
        {
            continue;
        }

        OrderedDetection orderedDetection;
        orderedDetection.detectionIndex = detectionIndex;
        orderedDetection.brightness = brightness;
        orderedDetection.reliability = reliability;
        orderedDetection.shapeScore = shapeScore;
        orderedDetection.score = brightGuidedDetectionPriorityScore(
            detection,
            brightness,
            reliability,
            shapeScore);
        orderedDetections.append(orderedDetection);
    }

    std::sort(orderedDetections.begin(), orderedDetections.end(), [](const OrderedDetection& lhs, const OrderedDetection& rhs) {
        if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
            return lhs.score > rhs.score;
        }
        return lhs.brightness > rhs.brightness;
    });
    const int orderedDetectionLimit = std::min(22, static_cast<int>(orderedDetections.size()));
    const int orderedCatalogLimit = std::min(24, static_cast<int>(anchorCatalogStars.size()));

    struct OrderedTriangleCandidate
    {
        std::array<int, 3> detectionIndices{{-1, -1, -1}};
        std::array<int, 3> catalogAnchorIndices{{-1, -1, -1}};
        double score = 0.0;
        double seedFov = 0.0;
        double similarityRmsPixels = 0.0;
        double brightnessMagnitudeError = 0.0;
    };
    QVector<OrderedTriangleCandidate> orderedTriangleCandidates;
    orderedTriangleCandidates.reserve(768);
    qint64 orderedTriangleCandidateCount = 0;
    qint64 orderedTriangleRatioMatches = 0;
    const double orderedRatioTolerance = 0.075;
    const double orderedSimilarityTolerancePixels = std::max(
        static_cast<double>(settings.m_plateSolveMatchRadius) * 1.25,
        28.0);

    // The catalog-triple magnitude ordering and triangle signature are invariant across
    // the detection-triple loop below, which used to recompute them for every detection
    // triple (~1500x each, ~3M sorts/signatures on rich fields). Precompute them once,
    // in the exact (i,j,k) iteration order so candidate encounter order — and therefore
    // the kept-768 cap behaviour — is unchanged.
    struct PrecomputedCatalogTriple
    {
        std::array<int, 3> order{{-1, -1, -1}};
        TriangleSignature signature;
        bool usable = false;
    };
    QVector<PrecomputedCatalogTriple> precomputedCatalogTriples;
    precomputedCatalogTriples.reserve(
        (orderedCatalogLimit * (orderedCatalogLimit - 1) * (orderedCatalogLimit - 2)) / 6);
    for (int i = 0; i < orderedCatalogLimit; ++i)
    {
        for (int j = i + 1; j < orderedCatalogLimit; ++j)
        {
            for (int k = j + 1; k < orderedCatalogLimit; ++k)
            {
                PrecomputedCatalogTriple triple;
                std::array<int, 3> catalogOrder {{i, j, k}};
                std::sort(catalogOrder.begin(), catalogOrder.end(), [&anchorCatalogStars](int lhs, int rhs) {
                    const double lhsMagnitude = anchorCatalogStars[lhs].visibleStar.magnitude;
                    const double rhsMagnitude = anchorCatalogStars[rhs].visibleStar.magnitude;
                    if (!qFuzzyCompare(lhsMagnitude + 1.0, rhsMagnitude + 1.0)) {
                        return lhsMagnitude < rhsMagnitude;
                    }
                    return anchorCatalogStars[lhs].visibleStar.catalogIndex
                        < anchorCatalogStars[rhs].visibleStar.catalogIndex;
                });
                triple.order = catalogOrder;
                const std::array<QPointF, 3> orderedCatalogProjectedPoints {{
                    anchorCatalogStars[catalogOrder[0]].projectedPoint,
                    anchorCatalogStars[catalogOrder[1]].projectedPoint,
                    anchorCatalogStars[catalogOrder[2]].projectedPoint
                }};
                triple.signature = buildTriangleSignature(orderedCatalogProjectedPoints);
                triple.usable = triple.signature.longestDistance >= 10.0;
                precomputedCatalogTriples.append(triple);
            }
        }
    }

    for (int a = 0; a < orderedDetectionLimit; ++a)
    {
        if (isCancellationRequested()) {
            return seeds;
        }
        for (int b = a + 1; b < orderedDetectionLimit; ++b)
        {
            for (int c = b + 1; c < orderedDetectionLimit; ++c)
            {
                std::array<int, 3> detectionOrder {{
                    orderedDetections[a].detectionIndex,
                    orderedDetections[b].detectionIndex,
                    orderedDetections[c].detectionIndex
                }};
                std::sort(detectionOrder.begin(), detectionOrder.end(), [this, &starDetections](int lhs, int rhs) {
                    const double lhsBrightness = cachedDetectionBrightnessMetric(starDetections, lhs);
                    const double rhsBrightness = cachedDetectionBrightnessMetric(starDetections, rhs);
                    if (!qFuzzyCompare(lhsBrightness + 1.0, rhsBrightness + 1.0)) {
                        return lhsBrightness > rhsBrightness;
                    }
                    return cachedDetectionReliabilityMetric(starDetections, lhs)
                        > cachedDetectionReliabilityMetric(starDetections, rhs);
                });
                const std::array<QPointF, 3> detectionPoints {{
                    starDetections[detectionOrder[0]].m_center,
                    starDetections[detectionOrder[1]].m_center,
                    starDetections[detectionOrder[2]].m_center
                }};
                const TriangleSignature detectionSignature = buildTriangleSignature(detectionPoints);
                if (detectionSignature.longestDistance < 20.0) {
                    continue;
                }

                for (const PrecomputedCatalogTriple& precomputedTriple : precomputedCatalogTriples)
                {
                    {
                        {
                            ++orderedTriangleCandidateCount;
                            if (!precomputedTriple.usable) {
                                continue;
                            }
                            const std::array<int, 3>& catalogOrder = precomputedTriple.order;
                            const TriangleSignature& catalogSignature = precomputedTriple.signature;

                            const double ratioError = std::fabs(detectionSignature.ratioShortToLong - catalogSignature.ratioShortToLong)
                                + std::fabs(detectionSignature.ratioMidToLong - catalogSignature.ratioMidToLong);
                            if (ratioError > orderedRatioTolerance) {
                                continue;
                            }
                            ++orderedTriangleRatioMatches;

                            const std::array<VisibleCatalogStar, 3> orderedStars {{
                                anchorCatalogStars[catalogOrder[0]].visibleStar,
                                anchorCatalogStars[catalogOrder[1]].visibleStar,
                                anchorCatalogStars[catalogOrder[2]].visibleStar
                            }};
                            const double catalogMagnitudeSum =
                                orderedStars[0].magnitude + orderedStars[1].magnitude + orderedStars[2].magnitude;
                            const double shapePenalty = std::max(0.0, 20.0 - narrowGuidedAnchorShapeScore(starDetections[detectionOrder[0]]))
                                + std::max(0.0, 20.0 - narrowGuidedAnchorShapeScore(starDetections[detectionOrder[1]]))
                                + std::max(0.0, 20.0 - narrowGuidedAnchorShapeScore(starDetections[detectionOrder[2]]));

                            for (const std::array<int, 3>& permutation : kTrianglePermutations)
                            {
                                if (!triangleBrightnessOrderCompatible(
                                        starDetections,
                                        detectionOrder,
                                        orderedStars,
                                        permutation))
                                {
                                    continue;
                                }
                                const double brightnessMagnitudeError = triangleBrightnessMagnitudeError(
                                    starDetections,
                                    detectionOrder,
                                    orderedStars,
                                    permutation);
                                if (!std::isfinite(brightnessMagnitudeError)
                                    || (brightnessMagnitudeError > 4.5))
                                {
                                    continue;
                                }

                                const std::array<int, 3> mappedCatalogOrder {{
                                    catalogOrder[permutation[0]],
                                    catalogOrder[permutation[1]],
                                    catalogOrder[permutation[2]]
                                }};
                                const std::array<QPointF, 3> catalogProjectedPoints {{
                                    anchorCatalogStars[mappedCatalogOrder[0]].projectedPoint,
                                    anchorCatalogStars[mappedCatalogOrder[1]].projectedPoint,
                                    anchorCatalogStars[mappedCatalogOrder[2]].projectedPoint
                                }};
                                double similarityScale = 1.0;
                                double similarityRotationDegrees = 0.0;
                                double similarityRmsPixels = std::numeric_limits<double>::infinity();
                                if (!estimateScreenSimilarity(
                                        catalogProjectedPoints,
                                        detectionPoints,
                                        similarityScale,
                                        similarityRotationDegrees,
                                        similarityRmsPixels)
                                    || (similarityRmsPixels > orderedSimilarityTolerancePixels))
                                {
                                    continue;
                                }
                                Q_UNUSED(similarityRotationDegrees)

                                const double seedFov = plateSolveStartUsesFov(settings)
                                    ? std::clamp(
                                        static_cast<double>(settings.m_fov),
                                        static_cast<double>(CameraSettings::m_minFov),
                                        static_cast<double>(CameraSettings::m_maxFov))
                                    : std::clamp(
                                        static_cast<double>(settings.m_fov) / std::max(0.50, std::min(2.0, similarityScale)),
                                        static_cast<double>(CameraSettings::m_minFov),
                                        static_cast<double>(CameraSettings::m_maxFov));
                                if (!seedFovCompatibleWithStartFov(settings, seedFov)) {
                                    continue;
                                }

                                const double fovPenalty = plateSolveStartUsesFov(settings)
                                    ? std::fabs(seedFov - static_cast<double>(settings.m_fov)) * 20.0
                                    : 0.0;
                                OrderedTriangleCandidate candidate;
                                candidate.detectionIndices = detectionOrder;
                                candidate.catalogAnchorIndices = mappedCatalogOrder;
                                candidate.seedFov = seedFov;
                                candidate.similarityRmsPixels = similarityRmsPixels;
                                candidate.brightnessMagnitudeError = brightnessMagnitudeError;
                                candidate.score = similarityRmsPixels * 4.0
                                    + ratioError * 160.0
                                    + catalogMagnitudeSum * 9.0
                                    + shapePenalty
                                    + fovPenalty
                                    + brightnessMagnitudeError * 55.0;
                                orderedTriangleCandidates.append(candidate);
                            }
                        }
                    }
                }
            }
        }
    }

    std::sort(orderedTriangleCandidates.begin(), orderedTriangleCandidates.end(), [](const OrderedTriangleCandidate& lhs, const OrderedTriangleCandidate& rhs) {
        if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
            return lhs.score < rhs.score;
        }
        if (!qFuzzyCompare(lhs.brightnessMagnitudeError + 1.0, rhs.brightnessMagnitudeError + 1.0)) {
            return lhs.brightnessMagnitudeError < rhs.brightnessMagnitudeError;
        }
        return lhs.similarityRmsPixels < rhs.similarityRmsPixels;
    });
    if (orderedTriangleCandidates.size() > 768) {
        orderedTriangleCandidates.resize(768);
    }

    qint64 orderedTriangleEvaluations = 0;
    qint64 orderedTriangleSeedValid = 0;
    qint64 orderedTriangleAnchorSupported = 0;
    qint64 orderedTriangleSeeds = 0;
    qint64 orderedTriangleVerifiedSeeds = 0;
    int orderedTriangleMaxAnchors = 0;
    const int orderedTriangleEvaluationLimit = 1500;
    const std::array<double, 3> orderedFovScales = plateSolveStartUsesFov(settings)
        ? std::array<double, 3>{{1.0, 1.0, 1.0}}
        : std::array<double, 3>{{0.98, 1.0, 1.02}};
    for (const OrderedTriangleCandidate& orderedTriangleCandidate : orderedTriangleCandidates)
    {
        if (isCancellationRequested()
            || (orderedTriangleEvaluations >= orderedTriangleEvaluationLimit)
            || (seeds.size() >= 96))
        {
            break;
        }

        const std::array<VisibleCatalogStar, 3> orderedStars {{
            anchorCatalogStars[orderedTriangleCandidate.catalogAnchorIndices[0]].visibleStar,
            anchorCatalogStars[orderedTriangleCandidate.catalogAnchorIndices[1]].visibleStar,
            anchorCatalogStars[orderedTriangleCandidate.catalogAnchorIndices[2]].visibleStar
        }};
        const std::array<int, 3> anchorDetectionIndices = orderedTriangleCandidate.detectionIndices;
        const std::array<int, 3> anchorCatalogIndices {{
            orderedStars[0].catalogIndex,
            orderedStars[1].catalogIndex,
            orderedStars[2].catalogIndex
        }};
        const bool debugTargetTriangle =
            debugTriangleContainsCatalogStar(catalogContext, anchorCatalogIndices);
        QVector<int> allowedCatalogIndices {
            anchorCatalogIndices[0],
            anchorCatalogIndices[1],
            anchorCatalogIndices[2]
        };
        double previousSeedFov = -1.0;
        for (double fovScale : orderedFovScales)
        {
            if (isCancellationRequested()
                || (orderedTriangleEvaluations >= orderedTriangleEvaluationLimit)
                || (seeds.size() >= 96))
            {
                break;
            }

            const double seedFov = std::clamp(
                orderedTriangleCandidate.seedFov * fovScale,
                static_cast<double>(CameraSettings::m_minFov),
                static_cast<double>(CameraSettings::m_maxFov));
            if (std::fabs(seedFov - previousSeedFov) < 1e-9) {
                continue;
            }
            previousSeedFov = seedFov;
            if (!seedFovCompatibleWithStartFov(settings, seedFov)) {
                continue;
            }

            const SkyProjector baseProjector = createProjector(
                settings,
                imageSize,
                0.0,
                90.0,
                0.0,
                seedFov,
                fixedCenterOffsetX,
                fixedCenterOffsetY,
                fixedDistortionK1);
            if (!baseProjector.valid) {
                continue;
            }

            std::array<SkyVector, 3> sourceVectors;
            bool haveSourceVectors = true;
            for (int idx = 0; idx < 3; ++idx)
            {
                if (!unprojectPixelToVector(
                        baseProjector,
                        starDetections[anchorDetectionIndices[idx]].m_center,
                        sourceVectors[idx]))
                {
                    haveSourceVectors = false;
                    break;
                }
            }
            if (!haveSourceVectors) {
                continue;
            }

            const std::array<SkyVector, 3> targetVectors {{
                orderedStars[0].vector,
                orderedStars[1].vector,
                orderedStars[2].vector
            }};
            double seedAzimuth = 0.0;
            double seedElevation = 0.0;
            double seedRoll = 0.0;
            double triangleRmsAngularError = std::numeric_limits<double>::infinity();
            double triangleMaxAngularError = std::numeric_limits<double>::infinity();
            if (!poseFromThreeVectorPairs(
                    baseProjector,
                    sourceVectors,
                    targetVectors,
                    seedAzimuth,
                    seedElevation,
                    seedRoll,
                    &triangleRmsAngularError,
                    &triangleMaxAngularError))
            {
                continue;
            }
            Q_UNUSED(triangleRmsAngularError)
            Q_UNUSED(triangleMaxAngularError)

            const double directionDelta = std::acos(std::clamp(
                dot(vectorFromAltAz(seedAzimuth, seedElevation), vectorFromAltAz(settings.m_azimuth, settings.m_elevation)),
                -1.0,
                1.0)) * 180.0 / kPi;
            if (directionDelta > startDirectionMaxDelta) {
                continue;
            }

            Evaluation seededCandidate = evaluatePose(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                seedAzimuth,
                seedElevation,
                seedRoll,
                seedFov,
                &allowedCatalogIndices,
                fixedCenterOffsetX,
                fixedCenterOffsetY,
                fixedDistortionK1);
            ++orderedTriangleEvaluations;
            if (!seededCandidate.valid) {
                if (debugTargetTriangle)
                {
                    qCDebug(cameraPlateSolverLog).noquote() << "CameraPlateSolver: ordered triangle target seed invalid"
                        << debugTriangleAnchorSummary(
                            catalogContext,
                            starDetections,
                            anchorDetectionIndices,
                            anchorCatalogIndices);
                }
                continue;
            }
            ++orderedTriangleSeedValid;

            const double anchorDistancePixels = std::min(
                128.0,
                std::max(
                    static_cast<double>(settings.m_plateSolveMatchRadius),
                    static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 4.0));
            const int orderedAnchorSupport = countProjectedAnchorSupport(
                    settings,
                    catalogContext,
                    imageSize,
                    starDetections,
                    seededCandidate,
                    anchorDetectionIndices,
                    anchorCatalogIndices,
                    anchorDistancePixels);
                orderedTriangleMaxAnchors = std::max(orderedTriangleMaxAnchors, orderedAnchorSupport);
                if (debugTargetTriangle)
                {
                    qCDebug(cameraPlateSolverLog).noquote() << "CameraPlateSolver: ordered triangle target seed"
                        << "anchorSupport" << orderedAnchorSupport
                        << "matches" << seededCandidate.matchCount
                        << "rms" << seededCandidate.rmsErrorPixels
                        << "Az" << seededCandidate.azimuthDegrees
                        << "El" << seededCandidate.elevationDegrees
                        << "Roll" << seededCandidate.rollDegrees
                        << "FoV" << seededCandidate.fovDegrees
                        << debugTriangleAnchorSummary(
                            catalogContext,
                            starDetections,
                            anchorDetectionIndices,
                            anchorCatalogIndices);
                }
                if (orderedAnchorSupport < 3)
                {
                    continue;
                }
                ++orderedTriangleAnchorSupported;

                Evaluation candidate = evaluatePose(
                    settings,
                    catalogContext,
                    imageSize,
                    captureDateTimeUtc,
                    starDetections,
                    detectionIndices,
                    seededCandidate.azimuthDegrees,
                    seededCandidate.elevationDegrees,
                    seededCandidate.rollDegrees,
                    seededCandidate.fovDegrees,
                    nullptr,
                    fixedCenterOffsetX,
                    fixedCenterOffsetY,
                    fixedDistortionK1);
                candidate.anchored = true;
                candidate.guidedTriangle = true;
                candidate.anchorDetectionIndex = anchorDetectionIndices[0];
                candidate.anchorCatalogIndex = anchorCatalogIndices[0];
                candidate.secondaryAnchorDetectionIndex = anchorDetectionIndices[1];
                candidate.secondaryAnchorCatalogIndex = anchorCatalogIndices[1];
                candidate.tertiaryAnchorDetectionIndex = anchorDetectionIndices[2];
                candidate.tertiaryAnchorCatalogIndex = anchorCatalogIndices[2];

                const double guidedTriangleSeedRmsCap = std::min(
                    std::max(18.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.75),
                    static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.90);
                if (candidate.valid
                    && (candidate.matchCount >= std::max(4, settings.m_plateSolveMinMatches))
                    && (candidate.rmsErrorPixels <= guidedTriangleSeedRmsCap)
                    && hasAcceptableBrightnessConsistency(candidate))
                {
                    seeds.append(candidate);
                    ++orderedTriangleSeeds;
                    if (debugTargetTriangle) {
                        qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: ordered triangle target appended";
                    }
                }

                Evaluation verifiedCandidate = verifyBlindSeedCandidate(
                    settings,
                    catalogContext,
                    imageSize,
                    captureDateTimeUtc,
                    starDetections,
                    detectionIndices,
                    candidate);
                if (verifiedCandidate.valid)
                {
                    verifiedCandidate.anchored = true;
                    verifiedCandidate.guidedTriangle = true;
                    verifiedCandidate.anchorDetectionIndex = anchorDetectionIndices[0];
                    verifiedCandidate.anchorCatalogIndex = anchorCatalogIndices[0];
                    verifiedCandidate.secondaryAnchorDetectionIndex = anchorDetectionIndices[1];
                    verifiedCandidate.secondaryAnchorCatalogIndex = anchorCatalogIndices[1];
                    verifiedCandidate.tertiaryAnchorDetectionIndex = anchorDetectionIndices[2];
                    verifiedCandidate.tertiaryAnchorCatalogIndex = anchorCatalogIndices[2];
                    seeds.append(verifiedCandidate);
                    ++orderedTriangleSeeds;
                    ++orderedTriangleVerifiedSeeds;
                    if (debugTargetTriangle) {
                        qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: ordered triangle target verified appended";
                    }
                }
        }
    }
    recordProfileMetric(QStringLiteral("search.guidedOrderedTriangleCandidates"), orderedTriangleCandidateCount);
    recordProfileMetric(QStringLiteral("search.guidedOrderedTriangleRatioMatches"), orderedTriangleRatioMatches);
    recordProfileMetric(QStringLiteral("search.guidedOrderedTriangleKept"), orderedTriangleCandidates.size());
    recordProfileMetric(QStringLiteral("search.guidedOrderedTriangleEvaluations"), orderedTriangleEvaluations);
    recordProfileMetric(QStringLiteral("search.guidedOrderedTriangleSeedValid"), orderedTriangleSeedValid);
    recordProfileMetric(QStringLiteral("search.guidedOrderedTriangleAnchorSupported"), orderedTriangleAnchorSupported);
    recordProfileMetric(QStringLiteral("search.guidedOrderedTriangleMaxAnchors"), orderedTriangleMaxAnchors);
    recordProfileMetric(QStringLiteral("search.guidedOrderedTriangleSeeds"), orderedTriangleSeeds);
    recordProfileMetric(QStringLiteral("search.guidedOrderedTriangleVerifiedSeeds"), orderedTriangleVerifiedSeeds);

    // The triple loop below visits up to ~900k (a,b,c) combinations per run; anything
    // per-iteration that can be hoisted or precomputed pays for itself many times over.
    // Pairwise pixel distances and catalog angular separations are shared between all
    // triples containing the pair, so compute the O(N^2) tables once instead of
    // re-deriving them (sqrt/acos) inside the O(N^3) loop; the environment lookup for
    // the TRIPLE debug aid was likewise being made per combination.
    const bool debugTripleEnv = qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_TRIPLE");
    const int anchorCount = anchors.size();
    QVector<double> pairDetectionDistance(anchorCount * anchorCount, 0.0);
    QVector<double> pairCatalogAngularDistance(anchorCount * anchorCount, 0.0);
    for (int a = 0; a < anchorCount; ++a)
    {
        const QPointF& da = starDetections[anchors[a].detectionIndex].m_center;
        const SkyVector& va = anchors[a].catalogStar.vector;
        for (int b = a + 1; b < anchorCount; ++b)
        {
            const QPointF& db = starDetections[anchors[b].detectionIndex].m_center;
            const double dist = QLineF(da, db).length();
            pairDetectionDistance[a * anchorCount + b] = dist;
            pairDetectionDistance[b * anchorCount + a] = dist;
            const double angular = std::acos(std::clamp(
                dot(va, anchors[b].catalogStar.vector), -1.0, 1.0)) * 180.0 / kPi;
            pairCatalogAngularDistance[a * anchorCount + b] = angular;
            pairCatalogAngularDistance[b * anchorCount + a] = angular;
        }
    }
    const SkyVector seedDirectionVector = vectorFromAltAz(settings.m_azimuth, settings.m_elevation);

    for (int a = 0; a < anchors.size(); ++a)
    {
        if (isCancellationRequested()) {
            return seeds;
        }
        for (int b = a + 1; b < anchors.size(); ++b)
        {
            if ((anchors[a].detectionIndex == anchors[b].detectionIndex)
                || (anchors[a].catalogIndex == anchors[b].catalogIndex))
            {
                continue;
            }
            for (int c = b + 1; c < anchors.size(); ++c)
            {
                if ((anchors[a].detectionIndex == anchors[c].detectionIndex)
                    || (anchors[b].detectionIndex == anchors[c].detectionIndex)
                    || (anchors[a].catalogIndex == anchors[c].catalogIndex)
                    || (anchors[b].catalogIndex == anchors[c].catalogIndex))
                {
                    continue;
                }
                ++ratioCandidates;

                const std::array<double, 3> detectionRatios = sortedRatios({{
                    pairDetectionDistance[a * anchorCount + b],
                    pairDetectionDistance[a * anchorCount + c],
                    pairDetectionDistance[b * anchorCount + c]
                }});
                if (detectionRatios[2] < 20.0) {
                    continue;
                }

                const SkyVector& va = anchors[a].catalogStar.vector;
                const SkyVector& vb = anchors[b].catalogStar.vector;
                const SkyVector& vc = anchors[c].catalogStar.vector;
                const std::array<double, 3> catalogAngularDistances {{
                    pairCatalogAngularDistance[a * anchorCount + b],
                    pairCatalogAngularDistance[a * anchorCount + c],
                    pairCatalogAngularDistance[b * anchorCount + c]
                }};
                const std::array<double, 3> catalogRatios = sortedRatios(catalogAngularDistances);
                if (catalogRatios[2] <= 0.01) {
                    continue;
                }

                const double ratioError = std::fabs(detectionRatios[0] - catalogRatios[0])
                    + std::fabs(detectionRatios[1] - catalogRatios[1]);
                if (debugTripleEnv)
                {
                    const QByteArray tripleSpec = qgetenv("SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_TRIPLE");
                    const QStringList tripleNames = QString::fromUtf8(tripleSpec).split(QLatin1Char(','), Qt::SkipEmptyParts);
                    const std::array<int, 3> debugCatalogIndices {{anchors[a].catalogIndex, anchors[b].catalogIndex, anchors[c].catalogIndex}};
                    const std::array<int, 3> debugDetectionIndices {{anchors[a].detectionIndex, anchors[b].detectionIndex, anchors[c].detectionIndex}};
                    int matchedNames = 0;
                    for (const QString& name : tripleNames)
                    {
                        for (int catalogIndex : debugCatalogIndices)
                        {
                            if ((catalogIndex >= 0)
                                && (catalogIndex < catalogContext.catalogStars.size())
                                && catalogDisplayName(catalogContext.catalogStars[catalogIndex]).contains(name.trimmed(), Qt::CaseInsensitive))
                            {
                                ++matchedNames;
                                break;
                            }
                        }
                    }
                    if (matchedNames >= tripleNames.size())
                    {
                        qCDebug(cameraPlateSolverLog).noquote() << "CameraPlateSolver: TRIPLE candidate"
                            << debugTriangleAnchorSummary(catalogContext, starDetections, debugDetectionIndices, debugCatalogIndices)
                            << "detectionRatios" << detectionRatios[0] << detectionRatios[1] << detectionRatios[2]
                            << "catalogRatios" << catalogRatios[0] << catalogRatios[1] << catalogRatios[2]
                            << "ratioError" << ratioError
                            << "ratioTolerance" << ratioTolerance
                            << "passes" << (ratioError <= ratioTolerance);
                    }
                }
                if (ratioError > ratioTolerance) {
                    continue;
                }

                ++ratioMatches;
                const double baseFov = std::clamp(
                    catalogRatios[2] * static_cast<double>(std::max(imageSize.width(), imageSize.height())) / std::max(1.0, detectionRatios[2]),
                    static_cast<double>(CameraSettings::m_minFov),
                    static_cast<double>(CameraSettings::m_maxFov));
                if (!seedFovCompatibleWithStartFov(settings, baseFov)) {
                    continue;
                }

                const SkyVector triangleCenter = normalize({
                    va.x + vb.x + vc.x,
                    va.y + vb.y + vc.y,
                    va.z + vb.z + vc.z
                });
                if (length(triangleCenter) <= 0.0) {
                    continue;
                }
                const double centerDelta = std::acos(std::clamp(
                    dot(triangleCenter, seedDirectionVector), -1.0, 1.0)) * 180.0 / kPi;
                if (centerDelta > startDirectionMaxDelta) {
                    continue;
                }

                const double fovPenalty = plateSolveStartUsesFov(settings)
                    ? std::fabs(baseFov - static_cast<double>(settings.m_fov)) * 20.0
                    : 0.0;
                TriangleCandidate triangleCandidate;
                triangleCandidate.anchorIndices = {{a, b, c}};
                triangleCandidate.score = anchors[a].score + anchors[b].score + anchors[c].score + ratioError * 140.0 + fovPenalty;
                triangleCandidate.baseFov = baseFov;
                if (debugTripleEnv)
                {
                    const QByteArray tripleSpec = qgetenv("SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_TRIPLE");
                    const QStringList tripleNames = QString::fromUtf8(tripleSpec).split(QLatin1Char(','), Qt::SkipEmptyParts);
                    const std::array<int, 3> debugCatalogIndices2 {{anchors[a].catalogIndex, anchors[b].catalogIndex, anchors[c].catalogIndex}};
                    const std::array<int, 3> debugDetectionIndices2 {{anchors[a].detectionIndex, anchors[b].detectionIndex, anchors[c].detectionIndex}};
                    int matchedNames2 = 0;
                    for (const QString& name : tripleNames)
                    {
                        for (int catalogIndex : debugCatalogIndices2)
                        {
                            if ((catalogIndex >= 0)
                                && (catalogIndex < catalogContext.catalogStars.size())
                                && catalogDisplayName(catalogContext.catalogStars[catalogIndex]).contains(name.trimmed(), Qt::CaseInsensitive))
                            {
                                ++matchedNames2;
                                break;
                            }
                        }
                    }
                    if (matchedNames2 >= tripleNames.size())
                    {
                        qCDebug(cameraPlateSolverLog).noquote() << "CameraPlateSolver: TRIPLE appended"
                            << debugTriangleAnchorSummary(catalogContext, starDetections, debugDetectionIndices2, debugCatalogIndices2)
                            << "score" << triangleCandidate.score
                            << "baseFov" << baseFov
                            << "centerDelta" << centerDelta
                            << "fovPenalty" << fovPenalty
                            << "anchorScores" << anchors[a].score << anchors[b].score << anchors[c].score;
                    }
                }
                triangleCandidates.append(triangleCandidate);
            }
        }
    }

    if (triangleCandidates.isEmpty()) {
        recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleAnchors"), anchors.size());
        recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleRatioCandidates"), ratioCandidates);
        recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleRatioMatches"), ratioMatches);
        return seeds;
    }

    std::sort(triangleCandidates.begin(), triangleCandidates.end(), [](const TriangleCandidate& lhs, const TriangleCandidate& rhs) {
        if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
            return lhs.score < rhs.score;
        }
        return lhs.baseFov < rhs.baseFov;
    });
    if (triangleCandidates.size() > 2048) {
        triangleCandidates.resize(2048);
    }
    recordProfileTiming(QStringLiteral("search.guidedAnchorTriangleGen"), anchorTriangleTimer.elapsed());
    const qint64 anchorTriangleEvalStartMs = anchorTriangleTimer.elapsed();

    const std::array<double, 3> seedFovScales = {{0.96, 1.0, 1.04}};
    qint64 seedEvaluations = 0;
    qint64 verifiedSeeds = 0;
    const int seedEvaluationLimit = 4000;
    const int seedLimit = 64;

    for (const TriangleCandidate& triangleCandidate : triangleCandidates)
    {
        if (isCancellationRequested() || (seedEvaluations >= seedEvaluationLimit) || (seeds.size() >= seedLimit)) {
            break;
        }

        std::array<AnchorCandidate, 3> triangleAnchors {{
            anchors[triangleCandidate.anchorIndices[0]],
            anchors[triangleCandidate.anchorIndices[1]],
            anchors[triangleCandidate.anchorIndices[2]]
        }};
        std::array<int, 3> anchorDetectionIndices {{
            triangleAnchors[0].detectionIndex,
            triangleAnchors[1].detectionIndex,
            triangleAnchors[2].detectionIndex
        }};
        std::array<int, 3> anchorCatalogIndices {{
            triangleAnchors[0].catalogIndex,
            triangleAnchors[1].catalogIndex,
            triangleAnchors[2].catalogIndex
        }};
        const bool debugTargetTriangle =
            debugTriangleContainsCatalogStar(catalogContext, anchorCatalogIndices);
        QVector<int> allowedCatalogIndices {
            anchorCatalogIndices[0],
            anchorCatalogIndices[1],
            anchorCatalogIndices[2]
        };

        for (double fovScale : seedFovScales)
        {
            if (isCancellationRequested() || (seedEvaluations >= seedEvaluationLimit) || (seeds.size() >= seedLimit)) {
                break;
            }
            const double seedFov = std::clamp(
                (plateSolveStartUsesFov(settings) ? static_cast<double>(settings.m_fov) : triangleCandidate.baseFov) * fovScale,
                static_cast<double>(CameraSettings::m_minFov),
                static_cast<double>(CameraSettings::m_maxFov));
            const SkyProjector baseProjector = createProjector(
                settings,
                imageSize,
                0.0,
                90.0,
                0.0,
                seedFov,
                fixedCenterOffsetX,
                fixedCenterOffsetY,
                fixedDistortionK1);
            if (!baseProjector.valid) {
                continue;
            }

            std::array<SkyVector, 3> sourceVectors;
            bool haveSourceVectors = true;
            for (int idx = 0; idx < 3; ++idx)
            {
                if (!unprojectPixelToVector(
                        baseProjector,
                        starDetections[anchorDetectionIndices[idx]].m_center,
                        sourceVectors[idx]))
                {
                    haveSourceVectors = false;
                    break;
                }
            }
            if (!haveSourceVectors) {
                continue;
            }

            std::array<SkyVector, 3> targetVectors {{
                triangleAnchors[0].catalogStar.vector,
                triangleAnchors[1].catalogStar.vector,
                triangleAnchors[2].catalogStar.vector
            }};
            double seedAzimuth = 0.0;
            double seedElevation = 0.0;
            double seedRoll = 0.0;
            double triangleRmsAngularError = std::numeric_limits<double>::infinity();
            double triangleMaxAngularError = std::numeric_limits<double>::infinity();
            if (!poseFromThreeVectorPairs(
                    baseProjector,
                    sourceVectors,
                    targetVectors,
                    seedAzimuth,
                    seedElevation,
                    seedRoll,
                    &triangleRmsAngularError,
                    &triangleMaxAngularError))
            {
                continue;
            }
            Q_UNUSED(triangleRmsAngularError)
            Q_UNUSED(triangleMaxAngularError)

            const double directionDelta = std::acos(std::clamp(
                dot(vectorFromAltAz(seedAzimuth, seedElevation), vectorFromAltAz(settings.m_azimuth, settings.m_elevation)),
                -1.0,
                1.0)) * 180.0 / kPi;
            if (directionDelta > startDirectionMaxDelta) {
                continue;
            }

            const Evaluation seededCandidate = evaluatePose(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                seedAzimuth,
                seedElevation,
                seedRoll,
                seedFov,
                &allowedCatalogIndices,
                fixedCenterOffsetX,
                fixedCenterOffsetY,
                fixedDistortionK1);
            ++seedEvaluations;
            if (!seededCandidate.valid) {
                if (debugTargetTriangle)
                {
                    qCDebug(cameraPlateSolverLog).noquote() << "CameraPlateSolver: anchor triangle target seed invalid"
                        << debugTriangleAnchorSummary(
                            catalogContext,
                            starDetections,
                            anchorDetectionIndices,
                            anchorCatalogIndices);
                }
                continue;
            }

            const double anchorDistancePixels = std::min(
                static_cast<double>(settings.m_plateSolveMatchRadius),
                std::max(6.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius)));
            const int anchorSupport = countProjectedAnchorSupport(
                    settings,
                    catalogContext,
                    imageSize,
                    starDetections,
                    seededCandidate,
                    anchorDetectionIndices,
                    anchorCatalogIndices,
                    anchorDistancePixels);
            if (debugTargetTriangle)
            {
                qCDebug(cameraPlateSolverLog).noquote() << "CameraPlateSolver: anchor triangle target seed"
                    << "anchorSupport" << anchorSupport
                    << "matches" << seededCandidate.matchCount
                    << "rms" << seededCandidate.rmsErrorPixels
                    << "Az" << seededCandidate.azimuthDegrees
                    << "El" << seededCandidate.elevationDegrees
                    << "Roll" << seededCandidate.rollDegrees
                    << "FoV" << seededCandidate.fovDegrees
                    << debugTriangleAnchorSummary(
                        catalogContext,
                        starDetections,
                        anchorDetectionIndices,
                        anchorCatalogIndices);
            }
            if (anchorSupport < 3)
            {
                continue;
            }

            Evaluation candidate = evaluatePose(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                seededCandidate.azimuthDegrees,
                seededCandidate.elevationDegrees,
                seededCandidate.rollDegrees,
                seededCandidate.fovDegrees,
                nullptr,
                fixedCenterOffsetX,
                fixedCenterOffsetY,
                fixedDistortionK1);
            candidate.anchored = true;
            candidate.guidedTriangle = true;
            candidate.anchorDetectionIndex = anchorDetectionIndices[0];
            candidate.anchorCatalogIndex = anchorCatalogIndices[0];
            candidate.secondaryAnchorDetectionIndex = anchorDetectionIndices[1];
            candidate.secondaryAnchorCatalogIndex = anchorCatalogIndices[1];
            candidate.tertiaryAnchorDetectionIndex = anchorDetectionIndices[2];
            candidate.tertiaryAnchorCatalogIndex = anchorCatalogIndices[2];

            if (candidate.valid
                && (candidate.matchCount >= std::max(3, settings.m_plateSolveMinMatches)))
            {
                seeds.append(candidate);
                if (debugTargetTriangle) {
                    qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: anchor triangle target appended";
                }
            }

            Evaluation verifiedCandidate = verifyBlindSeedCandidate(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                candidate);
            if (verifiedCandidate.valid)
            {
                verifiedCandidate.anchored = true;
                verifiedCandidate.guidedTriangle = true;
                verifiedCandidate.anchorDetectionIndex = anchorDetectionIndices[0];
                verifiedCandidate.anchorCatalogIndex = anchorCatalogIndices[0];
                verifiedCandidate.secondaryAnchorDetectionIndex = anchorDetectionIndices[1];
                verifiedCandidate.secondaryAnchorCatalogIndex = anchorCatalogIndices[1];
                verifiedCandidate.tertiaryAnchorDetectionIndex = anchorDetectionIndices[2];
                verifiedCandidate.tertiaryAnchorCatalogIndex = anchorCatalogIndices[2];
                seeds.append(verifiedCandidate);
                ++verifiedSeeds;
                if (debugTargetTriangle) {
                    qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: anchor triangle target verified appended";
                }
            }
        }
    }

    recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleAnchors"), anchors.size());
    recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleRatioCandidates"), ratioCandidates);
    recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleRatioMatches"), ratioMatches);
    recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleCandidates"), triangleCandidates.size());
    recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleSeedEvaluations"), seedEvaluations);
    recordProfileMetric(QStringLiteral("search.guidedAnchorTriangleVerifiedSeeds"), verifiedSeeds);
    recordProfileTiming(QStringLiteral("search.guidedAnchorTriangleEval"),
        anchorTriangleTimer.elapsed() - anchorTriangleEvalStartMs);

    std::sort(seeds.begin(), seeds.end(), [this](const Evaluation& lhs, const Evaluation& rhs) {
        return isBetterWeakModeEvaluation(lhs, rhs);
    });
    if (seeds.size() > seedLimit) {
        seeds.resize(seedLimit);
    }
    return seeds;
}

QVector<CameraPlateSolver::SolverContext::Evaluation> CameraPlateSolver::SolverContext::buildBrightPairSeeds(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const QVector<VisibleCatalogStar>& visibleStars)
{
    QVector<Evaluation> seeds;
    const bool useNarrowGuidedPairSeeds = plateSolveStartUsesDirection(settings)
        && !isWidePlateSolveContext(settings)
        && (isNarrowField(settings));
    if (isCancellationRequested()
        || (!isWidePlateSolveContext(settings) && !useNarrowGuidedPairSeeds)
        || (starDetections.size() < settings.m_plateSolveMinMatches)
        || (visibleStars.size() < settings.m_plateSolveMinMatches))
    {
        return seeds;
    }

    QVector<int> brightDetectionIndices = selectDetectionIndicesForBlindSignatures(
        starDetections,
        detectionIndices,
        10,
        10,
        16);
    if (useNarrowGuidedPairSeeds)
    {
        QSet<int> brightDetectionSeen;
        brightDetectionSeen.reserve(brightDetectionIndices.size() + 64);
        for (int detectionIndex : brightDetectionIndices) {
            brightDetectionSeen.insert(detectionIndex);
        }

        QVector<int> broadStarLikeDetections;
        broadStarLikeDetections.reserve(starDetections.size());
        for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex)
        {
            const CameraPipelineStarDetection& detection = starDetections[detectionIndex];
            if (detection.m_hotPixelSuspect) {
                continue;
            }
            if ((detection.m_saturated && (detection.m_fwhm >= 3.0f))
                || (narrowGuidedAnchorShapeScore(detection) >= 35.0))
            {
                broadStarLikeDetections.append(detectionIndex);
            }
        }
        std::sort(broadStarLikeDetections.begin(), broadStarLikeDetections.end(), [this, &starDetections](int lhs, int rhs) {
            const double lhsScore = narrowGuidedAnchorShapeScore(starDetections[lhs])
                + std::min(30.0, std::log1p(cachedDetectionBrightnessMetric(starDetections, lhs)) * 3.0)
                + std::min(20.0, std::log1p(cachedDetectionReliabilityMetric(starDetections, lhs)) * 5.0);
            const double rhsScore = narrowGuidedAnchorShapeScore(starDetections[rhs])
                + std::min(30.0, std::log1p(cachedDetectionBrightnessMetric(starDetections, rhs)) * 3.0)
                + std::min(20.0, std::log1p(cachedDetectionReliabilityMetric(starDetections, rhs)) * 5.0);
            if (!qFuzzyCompare(lhsScore + 1.0, rhsScore + 1.0)) {
                return lhsScore > rhsScore;
            }
            return cachedDetectionBrightnessMetric(starDetections, lhs)
                > cachedDetectionBrightnessMetric(starDetections, rhs);
        });
        const int broadDetectionLimit = std::min(64, static_cast<int>(broadStarLikeDetections.size()));
        for (int i = 0; i < broadDetectionLimit; ++i)
        {
            const int detectionIndex = broadStarLikeDetections[i];
            if (brightDetectionSeen.contains(detectionIndex)) {
                continue;
            }
            brightDetectionSeen.insert(detectionIndex);
            brightDetectionIndices.append(detectionIndex);
        }
    }
    if (brightDetectionIndices.size() < 2) {
        return seeds;
    }

    QVector<VisibleCatalogStar> brightCatalogStars;
    const double brightCatalogMaxMagnitude = useNarrowGuidedPairSeeds
        ? std::min(settings.m_plateSolveMaxMagnitude, kNarrowGuidedBrightCatalogMaxMagnitude)
        : kWideFovBrightFirstPassMaxMagnitude;
    const int brightCatalogLimit = useNarrowGuidedPairSeeds ? 96 : 20;
    brightCatalogStars.reserve(brightCatalogLimit);
    QSet<int> brightCatalogSeen;
    brightCatalogSeen.reserve(brightCatalogLimit + 32);
    for (const VisibleCatalogStar& visibleStar : visibleStars)
    {
        if (visibleStar.magnitude > brightCatalogMaxMagnitude) {
            break;
        }
        brightCatalogStars.append(visibleStar);
        brightCatalogSeen.insert(visibleStar.catalogIndex);
        if (brightCatalogStars.size() >= brightCatalogLimit) {
            break;
        }
    }
    if (useNarrowGuidedPairSeeds)
    {
        const int namedCatalogExtraLimit = 256;
        int namedCatalogExtraCount = 0;
        const auto appendNamedCatalogStar = [&](const VisibleCatalogStar& visibleStar) {
            if (visibleStar.magnitude > brightCatalogMaxMagnitude
                || brightCatalogSeen.contains(visibleStar.catalogIndex)
                || (visibleStar.catalogIndex < 0)
                || (visibleStar.catalogIndex >= catalogContext.catalogStars.size()))
            {
                return false;
            }

            const QString name = catalogDisplayName(catalogContext.catalogStars[visibleStar.catalogIndex]);
            if (!name.startsWith(QStringLiteral("HIP "), Qt::CaseInsensitive)
                && !name.startsWith(QStringLiteral("HR "), Qt::CaseInsensitive)
                && !name.startsWith(QStringLiteral("HD "), Qt::CaseInsensitive))
            {
                return false;
            }

            brightCatalogStars.append(visibleStar);
            brightCatalogSeen.insert(visibleStar.catalogIndex);
            return true;
        };
        const SkyVector center = normalize(vectorFromAltAz(settings.m_azimuth, settings.m_elevation));
        const double coreRadiusDegrees = std::max(1.5, static_cast<double>(settings.m_fov) * 1.75);
        const double coreMinDot = std::cos(coreRadiusDegrees * kPi / 180.0);
        for (const VisibleCatalogStar& visibleStar : catalogContext.visibleStars)
        {
            if (visibleStar.magnitude > brightCatalogMaxMagnitude) {
                break;
            }
            if (dot(center, visibleStar.vector) < coreMinDot) {
                continue;
            }
            if (appendNamedCatalogStar(visibleStar) && (++namedCatalogExtraCount >= namedCatalogExtraLimit)) {
                break;
            }
        }
        for (const VisibleCatalogStar& visibleStar : visibleStars)
        {
            if (visibleStar.magnitude > brightCatalogMaxMagnitude) {
                break;
            }
            if (appendNamedCatalogStar(visibleStar) && (++namedCatalogExtraCount >= namedCatalogExtraLimit)) {
                break;
            }
        }
        if (namedCatalogExtraCount < namedCatalogExtraLimit)
        {
            const double localRadiusDegrees = std::max(
                static_cast<double>(settings.m_plateSolveAzElSearchRadius) + static_cast<double>(settings.m_fov) * 2.0,
                static_cast<double>(settings.m_fov) * 4.0);
            const double minDot = std::cos(localRadiusDegrees * kPi / 180.0);
            for (const VisibleCatalogStar& visibleStar : catalogContext.visibleStars)
            {
                if (visibleStar.magnitude > brightCatalogMaxMagnitude) {
                    continue;
                }
                if (dot(center, visibleStar.vector) < minDot) {
                    continue;
                }
                if (appendNamedCatalogStar(visibleStar) && (++namedCatalogExtraCount >= namedCatalogExtraLimit)) {
                    break;
                }
            }
        }
    }
    if (brightCatalogStars.size() < 2) {
        return seeds;
    }

    // Precompute the angular separation of every bright catalog-star pair once. The inner
    // seed loop below compares each detection pair's separation against every catalog pair's
    // separation; that catalog separation depends only on the catalog pair, not the detection
    // pair, yet the original code recomputed acos(dot(...)) for all ~N^2 catalog pairs on every
    // one of the ~thousands of detection pairs (hundreds of millions of acos calls on dense
    // narrow-field frames). Caching it turns the hot inner test into a single array lookup while
    // leaving the iteration order — and therefore the set/order of seeds produced — unchanged.
    const int brightCatalogCount = brightCatalogStars.size();
    QVector<double> catalogPairSeparationRadians(
        static_cast<qsizetype>(brightCatalogCount) * static_cast<qsizetype>(brightCatalogCount), 0.0);
    for (int i = 0; i < brightCatalogCount; ++i)
    {
        const SkyVector& iVector = brightCatalogStars[i].vector;
        for (int j = i + 1; j < brightCatalogCount; ++j)
        {
            const double separation = std::acos(std::clamp(
                dot(iVector, brightCatalogStars[j].vector), -1.0, 1.0));
            catalogPairSeparationRadians[static_cast<qsizetype>(i) * brightCatalogCount + j] = separation;
            catalogPairSeparationRadians[static_cast<qsizetype>(j) * brightCatalogCount + i] = separation;
        }
    }

    // Per-star neighbour index: for each catalog star, the other stars ordered by angular
    // separation, with the matching separations stored in a parallel array. The seed loop
    // accepts a catalog partner only when its separation lies within a tolerance window of the
    // current detection pair's separation; a binary search over this sorted index finds that
    // window directly instead of scanning all ~N partners per detection pair. Narrow guided
    // solves then rank the candidate catalog pairs so the limited seed budget is spent on
    // the brightest and most separation-consistent possibilities first, instead of on
    // arbitrary catalog-index order aliases.
    QVector<QVector<double>> catalogNeighborSortedSeparation(brightCatalogCount);
    QVector<QVector<int>> catalogNeighborSortedIndex(brightCatalogCount);
    {
        QVector<int> neighborOrder;
        for (int i = 0; i < brightCatalogCount; ++i)
        {
            const qsizetype rowBase = static_cast<qsizetype>(i) * brightCatalogCount;
            neighborOrder.clear();
            neighborOrder.reserve(brightCatalogCount - 1);
            for (int j = 0; j < brightCatalogCount; ++j)
            {
                if (j != i) {
                    neighborOrder.append(j);
                }
            }
            std::sort(neighborOrder.begin(), neighborOrder.end(), [&](int a, int b) {
                return catalogPairSeparationRadians[rowBase + a] < catalogPairSeparationRadians[rowBase + b];
            });
            QVector<double>& rowSeparations = catalogNeighborSortedSeparation[i];
            QVector<int>& rowPartners = catalogNeighborSortedIndex[i];
            rowSeparations.reserve(neighborOrder.size());
            rowPartners.reserve(neighborOrder.size());
            for (int j : neighborOrder)
            {
                rowSeparations.append(catalogPairSeparationRadians[rowBase + j]);
                rowPartners.append(j);
            }
        }
    }
    const bool useNarrowGuidedNoRoll = useNarrowGuidedPairSeeds && !plateSolveStartUsesRoll(settings);
    const bool useNarrowGuidedRadialPrior = useNarrowGuidedNoRoll
        && (static_cast<double>(settings.m_plateSolveAzElSearchRadius)
            <= std::max(1.0, static_cast<double>(settings.m_fov) * 1.25));
    SkyProjector radialProjector;
    QPointF radialCenter;
    double radialTolerancePixels = 0.0;
    QVector<double> radialDetectionRadii;
    QVector<double> brightCatalogRadialRadii;
    if (useNarrowGuidedRadialPrior)
    {
        const bool useStartLens = plateSolveStartUsesLens(settings);
        radialProjector = createProjector(
            settings,
            imageSize,
            settings.m_azimuth,
            settings.m_elevation,
            0.0,
            settings.m_fov,
            useStartLens ? settings.m_lensCenterOffsetX : 0.0,
            useStartLens ? settings.m_lensCenterOffsetY : 0.0,
            useStartLens ? settings.m_lensDistortionK1 : 0.0);
        radialCenter = projectorPrincipalPoint(radialProjector);
        radialTolerancePixels = std::max(
            static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 6.0,
            std::max(imageSize.width(), imageSize.height()) * 0.12);
        if (radialProjector.valid)
        {
            radialDetectionRadii.resize(starDetections.size());
            for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex) {
                radialDetectionRadii[detectionIndex] = pointDistancePixels(starDetections[detectionIndex].m_center, radialCenter);
            }

            brightCatalogRadialRadii.resize(brightCatalogStars.size());
            for (int catalogIndex = 0; catalogIndex < brightCatalogStars.size(); ++catalogIndex)
            {
                QPointF projectedPoint;
                brightCatalogRadialRadii[catalogIndex] = projectVector(radialProjector, brightCatalogStars[catalogIndex].vector, projectedPoint)
                    ? pointDistancePixels(projectedPoint, radialCenter)
                    : std::numeric_limits<double>::quiet_NaN();
            }
        }
    }

    const auto isRadiallyCompatibleWithGuidedStart = [&](int detectionIndex, int brightCatalogIndex) {
        if (!useNarrowGuidedRadialPrior || !radialProjector.valid) {
            return true;
        }
        if ((detectionIndex < 0)
            || (detectionIndex >= radialDetectionRadii.size())
            || (brightCatalogIndex < 0)
            || (brightCatalogIndex >= brightCatalogRadialRadii.size())
            || !std::isfinite(brightCatalogRadialRadii[brightCatalogIndex]))
        {
            return false;
        }
        return std::fabs(radialDetectionRadii[detectionIndex] - brightCatalogRadialRadii[brightCatalogIndex]) <= radialTolerancePixels;
    };

    if (useNarrowGuidedRadialPrior)
    {
        if (radialProjector.valid)
        {
            QSet<int> radialSeen;
            radialSeen.reserve(brightDetectionIndices.size() + brightCatalogStars.size() * 4);
            for (int detectionIndex : brightDetectionIndices) {
                radialSeen.insert(detectionIndex);
            }
            QHash<int, double> namedRadialAnchorScores;
            namedRadialAnchorScores.reserve(brightDetectionIndices.size() + 64);
            for (int catalogIndex = 0; catalogIndex < brightCatalogStars.size(); ++catalogIndex)
            {
                if ((catalogIndex >= brightCatalogRadialRadii.size())
                    || !std::isfinite(brightCatalogRadialRadii[catalogIndex]))
                {
                    continue;
                }
                const double projectedRadius = brightCatalogRadialRadii[catalogIndex];
                const bool namedCatalogStar = (brightCatalogStars[catalogIndex].catalogIndex >= 0)
                    && (brightCatalogStars[catalogIndex].catalogIndex < catalogContext.catalogStars.size())
                    && isNamedSparseGuidedCatalogStar(catalogContext.catalogStars[brightCatalogStars[catalogIndex].catalogIndex]);
                QVector<QPair<double, int>> radialCandidates;
                radialCandidates.reserve(starDetections.size());
                for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex)
                {
                    const CameraPipelineStarDetection& detection = starDetections[detectionIndex];
                    if (detection.m_hotPixelSuspect) {
                        continue;
                    }
                    const double reliability = cachedDetectionReliabilityMetric(starDetections, detectionIndex);
                    if (reliability <= 0.0) {
                        continue;
                    }
                    const double detectionRadius = radialDetectionRadii[detectionIndex];
                    const double radialError = std::fabs(detectionRadius - projectedRadius);
                    if (radialError > radialTolerancePixels) {
                        continue;
                    }
                    const double score = radialError
                        + detection.m_aspectRatio * 2.0
                        - std::min(12.0, std::log1p(reliability) * 1.2)
                        - std::min(30.0, narrowGuidedAnchorShapeScore(detection) * 0.35);
                    radialCandidates.append(qMakePair(score, detectionIndex));
                }
                std::sort(radialCandidates.begin(), radialCandidates.end(), [](const QPair<double, int>& lhs, const QPair<double, int>& rhs) {
                    if (!qFuzzyCompare(lhs.first + 1.0, rhs.first + 1.0)) {
                        return lhs.first < rhs.first;
                    }
                    return lhs.second < rhs.second;
                });
                const int perCatalogLimit = std::min(
                    namedCatalogStar ? 24 : 8,
                    static_cast<int>(radialCandidates.size()));
                for (int i = 0; i < perCatalogLimit; ++i)
                {
                    const int detectionIndex = radialCandidates[i].second;
                    if (namedCatalogStar)
                    {
                        const double namedScore = radialCandidates[i].first
                            + brightCatalogStars[catalogIndex].magnitude * 6.0;
                        const auto existingScoreIt = namedRadialAnchorScores.constFind(detectionIndex);
                        if ((existingScoreIt == namedRadialAnchorScores.cend())
                            || (namedScore < existingScoreIt.value()))
                        {
                            namedRadialAnchorScores.insert(detectionIndex, namedScore);
                        }
                    }
                    if (radialSeen.contains(detectionIndex)) {
                        continue;
                    }
                    radialSeen.insert(detectionIndex);
                    brightDetectionIndices.append(detectionIndex);
                }
            }
            if (!namedRadialAnchorScores.isEmpty())
            {
                QVector<QPair<double, int>> namedRadialAnchors;
                namedRadialAnchors.reserve(namedRadialAnchorScores.size());
                for (auto it = namedRadialAnchorScores.cbegin(); it != namedRadialAnchorScores.cend(); ++it) {
                    namedRadialAnchors.append(qMakePair(it.value(), it.key()));
                }
                std::sort(namedRadialAnchors.begin(), namedRadialAnchors.end(), [](const QPair<double, int>& lhs, const QPair<double, int>& rhs) {
                    if (!qFuzzyCompare(lhs.first + 1.0, rhs.first + 1.0)) {
                        return lhs.first < rhs.first;
                    }
                    return lhs.second < rhs.second;
                });

                QVector<int> reorderedBrightDetections;
                reorderedBrightDetections.reserve(brightDetectionIndices.size());
                QSet<int> reorderedSeen;
                reorderedSeen.reserve(brightDetectionIndices.size());
                for (const QPair<double, int>& namedAnchor : namedRadialAnchors)
                {
                    if (reorderedSeen.contains(namedAnchor.second)) {
                        continue;
                    }
                    reorderedSeen.insert(namedAnchor.second);
                    reorderedBrightDetections.append(namedAnchor.second);
                }
                for (int detectionIndex : brightDetectionIndices)
                {
                    if (reorderedSeen.contains(detectionIndex)) {
                        continue;
                    }
                    reorderedSeen.insert(detectionIndex);
                    reorderedBrightDetections.append(detectionIndex);
                }
                brightDetectionIndices = std::move(reorderedBrightDetections);
            }
        }

    }

    const int brightDetectionLimit = useNarrowGuidedPairSeeds ? 128 : 10;
    if (brightDetectionIndices.size() > brightDetectionLimit) {
        brightDetectionIndices.resize(brightDetectionLimit);
    }
    if (useNarrowGuidedPairSeeds
        && qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE"))
    {
        const int debugDetectionCount = std::min(48, static_cast<int>(brightDetectionIndices.size()));
        qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: bright-pair detection pool"
                 << brightDetectionIndices.size()
                 << "showing" << debugDetectionCount;
        for (int i = 0; i < debugDetectionCount; ++i)
        {
            const int detectionIndex = brightDetectionIndices[i];
            if ((detectionIndex < 0) || (detectionIndex >= starDetections.size())) {
                continue;
            }
            const CameraPipelineStarDetection& detection = starDetections[detectionIndex];
            qCDebug(cameraPlateSolverLog).noquote().nospace()
                << "CameraPlateSolver: bright detection #" << i
                << " idx=" << detectionIndex
                << " x=" << detection.m_center.x()
                << " y=" << detection.m_center.y()
                << " snr=" << detection.m_snr
                << " fwhm=" << detection.m_fwhm
                << " flux=" << detection.m_flux
                << " peak=" << detection.m_peakValue
                << " round=" << detection.m_roundness
                << " aspect=" << detection.m_aspectRatio
                << " sat=" << detection.m_saturated
                << " shape=" << narrowGuidedAnchorShapeScore(detection)
                << " bright=" << cachedDetectionBrightnessMetric(starDetections, detectionIndex)
                << " reliab=" << cachedDetectionReliabilityMetric(starDetections, detectionIndex);
        }
    }

    QVector<double> seedFovs;
    if (plateSolveStartUsesFov(settings))
    {
        for (double scale : {1.0, 0.96, 1.04})
        {
            seedFovs.append(std::clamp(
                static_cast<double>(settings.m_fov) * scale,
                static_cast<double>(CameraSettings::m_minFov),
                static_cast<double>(CameraSettings::m_maxFov)));
        }
    }
    else
    {
        for (double fovDegrees : {130.0, 160.0, 180.0}) {
            seedFovs.append(fovDegrees);
        }
    }

    const bool useStartLens = plateSolveStartUsesLens(settings);
    const double fixedCenterOffsetX = useStartLens ? settings.m_lensCenterOffsetX : 0.0;
    const double fixedCenterOffsetY = useStartLens ? settings.m_lensCenterOffsetY : 0.0;
    const double fixedDistortionK1 = useStartLens ? settings.m_lensDistortionK1 : 0.0;
    QVector<int> narrowPairSeedVerificationDetectionIndices;
    const QVector<int>* pairSeedVerificationDetectionIndices = &detectionIndices;
    if (useNarrowGuidedPairSeeds)
    {
        QSet<int> verificationSeen;
        verificationSeen.reserve(std::min(static_cast<int>(starDetections.size()), 320));
        const auto appendVerificationDetection = [&](int detectionIndex) {
            if ((detectionIndex < 0)
                || (detectionIndex >= starDetections.size())
                || verificationSeen.contains(detectionIndex)
                || starDetections[detectionIndex].m_hotPixelSuspect)
            {
                return;
            }
            verificationSeen.insert(detectionIndex);
            narrowPairSeedVerificationDetectionIndices.append(detectionIndex);
        };
        for (int detectionIndex : detectionIndices) {
            appendVerificationDetection(detectionIndex);
        }
        for (int detectionIndex : brightDetectionIndices) {
            appendVerificationDetection(detectionIndex);
        }

        QVector<int> rankedVerificationDetections;
        rankedVerificationDetections.reserve(starDetections.size());
        for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex)
        {
            if (!starDetections[detectionIndex].m_hotPixelSuspect) {
                rankedVerificationDetections.append(detectionIndex);
            }
        }
        std::sort(rankedVerificationDetections.begin(), rankedVerificationDetections.end(), [this, &starDetections](int lhs, int rhs) {
            const double lhsScore = cachedDetectionReliabilityMetric(starDetections, lhs)
                + std::min(80.0, cachedDetectionBrightnessMetric(starDetections, lhs) * 0.04)
                + narrowGuidedAnchorShapeScore(starDetections[lhs]) * 0.35;
            const double rhsScore = cachedDetectionReliabilityMetric(starDetections, rhs)
                + std::min(80.0, cachedDetectionBrightnessMetric(starDetections, rhs) * 0.04)
                + narrowGuidedAnchorShapeScore(starDetections[rhs]) * 0.35;
            if (!qFuzzyCompare(lhsScore + 1.0, rhsScore + 1.0)) {
                return lhsScore > rhsScore;
            }
            return cachedDetectionBrightnessMetric(starDetections, lhs)
                > cachedDetectionBrightnessMetric(starDetections, rhs);
        });
        const int verificationDetectionLimit = std::min(256, static_cast<int>(starDetections.size()));
        for (int detectionIndex : rankedVerificationDetections)
        {
            if (narrowPairSeedVerificationDetectionIndices.size() >= verificationDetectionLimit) {
                break;
            }
            appendVerificationDetection(detectionIndex);
        }
        pairSeedVerificationDetectionIndices = &narrowPairSeedVerificationDetectionIndices;
    }
    QVector<quint8> pairSeedBaseDetectionSelected(starDetections.size(), 0);
    for (int detectionIndex : detectionIndices)
    {
        if ((detectionIndex >= 0) && (detectionIndex < pairSeedBaseDetectionSelected.size())) {
            pairSeedBaseDetectionSelected[detectionIndex] = 1;
        }
    }
    QVector<double> brightDetectionBrightness;
    brightDetectionBrightness.reserve(brightDetectionIndices.size());
    for (int detectionIndex : brightDetectionIndices) {
        brightDetectionBrightness.append(cachedDetectionBrightnessMetric(starDetections, detectionIndex));
    }
    qint64 seedEvaluations = 0;
    qint64 verifiedSeeds = 0;
    const qint64 verifiedSeedLimit = useNarrowGuidedPairSeeds ? 192 : 12;
    bool reachedSeedLimit = false;
    bool cancelledSeedSearch = false;
    const auto shouldStopBrightPairSeedSearch = [&]() {
        return isCancellationRequested() || cancelledSeedSearch || reachedSeedLimit;
    };
    const auto appendVerifiedSeed = [&](const Evaluation& seed) {
        seeds.append(seed);
        ++verifiedSeeds;
        if (verifiedSeeds >= verifiedSeedLimit) {
            reachedSeedLimit = true;
        }
    };

    struct BrightPairSeedWorkResult
    {
        QVector<Evaluation> seeds;
        qint64 seedEvaluations = 0;
        bool cancelled = false;
    };

    struct BrightCatalogPairCandidate
    {
        int firstCatalog = -1;
        int secondCatalog = -1;
        double separationErrorRadians = 0.0;
        double score = 0.0;
    };

    const auto processBrightPairFirstDetection = [&](SolverContext& context,
                                                     const SkyProjector& baseProjector,
                                                     const QVector<SkyVector>& brightDetectionVectors,
                                                     const QVector<quint8>& brightDetectionVectorValid,
                                                     double seedFov,
                                                     int firstDetection) {
        BrightPairSeedWorkResult result;
        if ((firstDetection < 0) || (firstDetection >= brightDetectionIndices.size() - 1)) {
            return result;
        }
        if ((firstDetection >= brightDetectionVectorValid.size()) || !brightDetectionVectorValid[firstDetection]) {
            return result;
        }

        QVector<BrightCatalogPairCandidate> catalogPairCandidates;
        catalogPairCandidates.reserve(brightCatalogCount);
        QVector<int> windowSecondCatalogs;
        windowSecondCatalogs.reserve(brightCatalogCount);
        QVector<int> seedDetectionIndices;
        seedDetectionIndices.reserve(detectionIndices.size() + 2);
        QVector<int> allowedCatalogIndices(2);
        const qsizetype localSeedLimit = useNarrowGuidedPairSeeds ? 8 : verifiedSeedLimit;
        const qsizetype localPrioritySeedLimit = useNarrowGuidedPairSeeds ? 16 : 0;
        const qsizetype localHardSeedLimit = localSeedLimit + localPrioritySeedLimit;
        const qsizetype localSeedLimitPerDetectionPair = useNarrowGuidedPairSeeds ? 8 : verifiedSeedLimit;
        const qsizetype localPrioritySeedLimitPerDetectionPair = useNarrowGuidedPairSeeds ? 16 : 0;
        result.seeds.reserve(static_cast<int>(std::min<qsizetype>(localHardSeedLimit, 16)));
        const auto localStop = [&]() {
            return context.isCancellationRequested()
                || (result.seeds.size() >= localHardSeedLimit);
        };
        qsizetype localPrioritySeeds = 0;

        const int detectionIndexA = brightDetectionIndices[firstDetection];
        const double detectionBrightnessA = brightDetectionBrightness[firstDetection];
        const SkyVector& sourceA = brightDetectionVectors[firstDetection];
        const bool detectionASelected = (detectionIndexA >= 0)
            && (detectionIndexA < pairSeedBaseDetectionSelected.size())
            && pairSeedBaseDetectionSelected[detectionIndexA];
        const auto isPriorityCatalogPairCandidate = [&](int firstCatalog, int secondCatalog) {
            if (!useNarrowGuidedPairSeeds) {
                return false;
            }

            const auto isPriorityCatalogStar = [&](int brightCatalogIndex) {
                if ((brightCatalogIndex < 0) || (brightCatalogIndex >= brightCatalogStars.size())) {
                    return false;
                }
                const int sourceCatalogIndex = brightCatalogStars[brightCatalogIndex].catalogIndex;
                if ((sourceCatalogIndex < 0) || (sourceCatalogIndex >= catalogContext.catalogStars.size())) {
                    return false;
                }
                const CatalogStar& catalogStar = catalogContext.catalogStars[sourceCatalogIndex];
                return (catalogStar.magnitude <= std::min(
                            static_cast<double>(settings.m_plateSolveMaxMagnitude),
                            10.5))
                    && isNamedSparseGuidedCatalogStar(catalogStar);
            };

            return isPriorityCatalogStar(firstCatalog)
                && isPriorityCatalogStar(secondCatalog);
        };
        for (int secondDetection = firstDetection + 1; secondDetection < brightDetectionIndices.size(); ++secondDetection)
        {
            if (localStop()) {
                break;
            }
            if ((secondDetection >= brightDetectionVectorValid.size()) || !brightDetectionVectorValid[secondDetection]) {
                continue;
            }

            const int detectionIndexB = brightDetectionIndices[secondDetection];
            const double detectionBrightnessB = brightDetectionBrightness[secondDetection];
            const double detectionLogBrightnessRatio = std::fabs(std::log(
                (detectionBrightnessA + 1.0) / (detectionBrightnessB + 1.0)));
            const QVector<int>* seedDetectionIndicesPtr = &detectionIndices;
            const bool detectionBSelected = (detectionIndexB >= 0)
                && (detectionIndexB < pairSeedBaseDetectionSelected.size())
                && pairSeedBaseDetectionSelected[detectionIndexB];
            if (!detectionASelected || !detectionBSelected)
            {
                seedDetectionIndices = detectionIndices;
                if (!detectionASelected) {
                    seedDetectionIndices.append(detectionIndexA);
                }
                if (!detectionBSelected && (detectionIndexB != detectionIndexA)) {
                    seedDetectionIndices.append(detectionIndexB);
                }
                seedDetectionIndicesPtr = &seedDetectionIndices;
            }
            const QVector<int>& seedMatchDetectionIndices = *seedDetectionIndicesPtr;
            const SkyVector& sourceB = brightDetectionVectors[secondDetection];
            const double sourceSeparationRadians = std::acos(std::clamp(dot(sourceA, sourceB), -1.0, 1.0));
            const double minSourceSeparationDegrees = useNarrowGuidedPairSeeds ? 0.05 : 1.0;
            if (sourceSeparationRadians < degToRad(minSourceSeparationDegrees)) {
                continue;
            }
            qsizetype localDetectionPairSeeds = 0;
            qsizetype localDetectionPairPrioritySeeds = 0;
            const auto detectionPairHardStop = [&]() {
                return localStop()
                    || ((localDetectionPairSeeds >= localSeedLimitPerDetectionPair)
                        && (localDetectionPairPrioritySeeds >= localPrioritySeedLimitPerDetectionPair));
            };
            const auto appendDetectionPairSeed = [&](const Evaluation& seed, bool prioritySeed) {
                if (detectionPairHardStop()) {
                    return;
                }
                if (prioritySeed)
                {
                    if ((localPrioritySeeds >= localPrioritySeedLimit)
                        || (localDetectionPairPrioritySeeds >= localPrioritySeedLimitPerDetectionPair))
                    {
                        return;
                    }
                    result.seeds.append(seed);
                    ++localPrioritySeeds;
                    ++localDetectionPairPrioritySeeds;
                    return;
                }
                if ((localDetectionPairSeeds >= localSeedLimitPerDetectionPair)
                    || (result.seeds.size() >= localSeedLimit))
                {
                    return;
                }
                result.seeds.append(seed);
                ++localDetectionPairSeeds;
            };
            const double separationToleranceRadians = useNarrowGuidedPairSeeds
                ? std::max(degToRad(0.04), sourceSeparationRadians * 0.18)
                : plateSolveStartUsesFov(settings)
                    ? std::max(degToRad(2.0), sourceSeparationRadians * 0.18)
                    : std::max(degToRad(5.0), sourceSeparationRadians * 0.30);
            const double windowLowSeparation = sourceSeparationRadians - separationToleranceRadians - 1e-9;
            const double windowHighSeparation = sourceSeparationRadians + separationToleranceRadians + 1e-9;

            catalogPairCandidates.clear();
            for (int firstCatalog = 0; firstCatalog < brightCatalogStars.size(); ++firstCatalog)
            {
                if (localStop()) {
                    break;
                }

                const QVector<double>& rowSeparations = catalogNeighborSortedSeparation[firstCatalog];
                const QVector<int>& rowPartners = catalogNeighborSortedIndex[firstCatalog];
                const auto windowBegin = std::lower_bound(
                    rowSeparations.cbegin(), rowSeparations.cend(), windowLowSeparation);
                const auto windowEnd = std::upper_bound(
                    rowSeparations.cbegin(), rowSeparations.cend(), windowHighSeparation);
                windowSecondCatalogs.clear();
                for (auto it = windowBegin; it != windowEnd; ++it) {
                    windowSecondCatalogs.append(rowPartners[it - rowSeparations.cbegin()]);
                }

                for (int secondCatalog : windowSecondCatalogs)
                {
                    if (localStop()) {
                        break;
                    }
                    if (firstCatalog == secondCatalog) {
                        continue;
                    }
                    const double catalogSeparationRadians = catalogPairSeparationRadians[
                        static_cast<qsizetype>(firstCatalog) * brightCatalogCount + secondCatalog];
                    const double separationErrorRadians = std::fabs(sourceSeparationRadians - catalogSeparationRadians);
                    if (separationErrorRadians > separationToleranceRadians) {
                        continue;
                    }
                    if (useNarrowGuidedPairSeeds)
                    {
                        const double catalogMagnitudeDelta = std::fabs(
                            brightCatalogStars[firstCatalog].magnitude
                            - brightCatalogStars[secondCatalog].magnitude);
                        if (((detectionLogBrightnessRatio < 0.05) && (catalogMagnitudeDelta > 6.5))
                            || ((detectionLogBrightnessRatio > 3.0) && (catalogMagnitudeDelta < 0.05)))
                        {
                            continue;
                        }
                    }
                    if (!isRadiallyCompatibleWithGuidedStart(detectionIndexA, firstCatalog)
                        || !isRadiallyCompatibleWithGuidedStart(detectionIndexB, secondCatalog))
                    {
                        continue;
                    }

                    const double firstMagnitude = brightCatalogStars[firstCatalog].magnitude;
                    const double secondMagnitude = brightCatalogStars[secondCatalog].magnitude;
                    double score = static_cast<double>(firstCatalog) * 1e-6
                        + static_cast<double>(secondCatalog) * 1e-9;
                    if (useNarrowGuidedPairSeeds)
                    {
                        const double signedDetectionLogBrightnessRatio = std::log(
                            (detectionBrightnessA + 1.0) / (detectionBrightnessB + 1.0));
                        const double signedCatalogLogBrightnessRatio = -0.9210340371976183
                            * (firstMagnitude - secondMagnitude);
                        const double catalogMagnitudeDelta = std::fabs(firstMagnitude - secondMagnitude);
                        const bool firstNamed = (brightCatalogStars[firstCatalog].catalogIndex >= 0)
                            && (brightCatalogStars[firstCatalog].catalogIndex < catalogContext.catalogStars.size())
                            && isNamedSparseGuidedCatalogStar(catalogContext.catalogStars[brightCatalogStars[firstCatalog].catalogIndex]);
                        const bool secondNamed = (brightCatalogStars[secondCatalog].catalogIndex >= 0)
                            && (brightCatalogStars[secondCatalog].catalogIndex < catalogContext.catalogStars.size())
                            && isNamedSparseGuidedCatalogStar(catalogContext.catalogStars[brightCatalogStars[secondCatalog].catalogIndex]);
                        score = (separationErrorRadians / std::max(degToRad(0.01), separationToleranceRadians)) * 80.0
                            + (firstMagnitude + secondMagnitude) * 1.6
                            + catalogMagnitudeDelta * 0.12
                            + std::fabs(signedDetectionLogBrightnessRatio - signedCatalogLogBrightnessRatio) * 2.0
                            - (firstNamed ? 8.0 : 0.0)
                            - (secondNamed ? 8.0 : 0.0);
                    }
                    catalogPairCandidates.append({firstCatalog, secondCatalog, separationErrorRadians, score});
                }
            }

            std::sort(catalogPairCandidates.begin(), catalogPairCandidates.end(),
                [](const BrightCatalogPairCandidate& lhs, const BrightCatalogPairCandidate& rhs) {
                    if (!qFuzzyCompare(lhs.score + 1.0, rhs.score + 1.0)) {
                        return lhs.score < rhs.score;
                    }
                    if (!qFuzzyCompare(lhs.separationErrorRadians + 1.0, rhs.separationErrorRadians + 1.0)) {
                        return lhs.separationErrorRadians < rhs.separationErrorRadians;
                    }
                    if (lhs.firstCatalog != rhs.firstCatalog) {
                        return lhs.firstCatalog < rhs.firstCatalog;
                    }
                    return lhs.secondCatalog < rhs.secondCatalog;
                });

            const int maxCatalogPairCandidates = useNarrowGuidedPairSeeds
                ? std::min(static_cast<int>(catalogPairCandidates.size()), 192)
                : static_cast<int>(catalogPairCandidates.size());
            for (int catalogPairIndex = 0; catalogPairIndex < maxCatalogPairCandidates; ++catalogPairIndex)
            {
                if (detectionPairHardStop()) {
                    break;
                }
                const BrightCatalogPairCandidate& catalogPairCandidate = catalogPairCandidates[catalogPairIndex];
                const int firstCatalog = catalogPairCandidate.firstCatalog;
                const int secondCatalog = catalogPairCandidate.secondCatalog;
                const bool priorityCatalogPair = isPriorityCatalogPairCandidate(firstCatalog, secondCatalog);
                if ((localDetectionPairSeeds >= localSeedLimitPerDetectionPair)
                    && !priorityCatalogPair)
                {
                    continue;
                }
                if ((result.seeds.size() >= localSeedLimit)
                    && !priorityCatalogPair)
                {
                    continue;
                }
                double seedAzimuth = 0.0;
                double seedElevation = 0.0;
                double seedRoll = 0.0;
                if (!poseFromTwoVectorPairs(
                        baseProjector,
                        sourceA,
                        sourceB,
                        brightCatalogStars[firstCatalog].vector,
                        brightCatalogStars[secondCatalog].vector,
                        seedAzimuth,
                        seedElevation,
                        seedRoll))
                {
                    continue;
                }

                    allowedCatalogIndices[0] = brightCatalogStars[firstCatalog].catalogIndex;
                    allowedCatalogIndices[1] = brightCatalogStars[secondCatalog].catalogIndex;
                    const Evaluation seededCandidate = context.evaluatePose(
                        settings,
                        catalogContext,
                        imageSize,
                        captureDateTimeUtc,
                        starDetections,
                        seedMatchDetectionIndices,
                        seedAzimuth,
                        seedElevation,
                        seedRoll,
                        seedFov,
                        &allowedCatalogIndices,
                        fixedCenterOffsetX,
                        fixedCenterOffsetY,
                        fixedDistortionK1);
                    ++result.seedEvaluations;
                    if (!seededCandidate.valid) {
                        continue;
                    }
                    const std::array<int, 2> anchorDetectionIndices {{
                        detectionIndexA,
                        detectionIndexB
                    }};
                    const std::array<int, 2> anchorCatalogIndices {{
                        brightCatalogStars[firstCatalog].catalogIndex,
                        brightCatalogStars[secondCatalog].catalogIndex
                    }};
                    const bool hasAnchorSupport = context.hasSeedAnchorSupport(seededCandidate, anchorDetectionIndices, anchorCatalogIndices, 2);
                    if (!hasAnchorSupport) {
                        continue;
                    }

                    Evaluation sparsePairCandidate = seededCandidate;
                    sparsePairCandidate.sparseGuidedPair = useNarrowGuidedPairSeeds;
                    sparsePairCandidate.anchored = useNarrowGuidedPairSeeds;
                    sparsePairCandidate.anchorDetectionIndex = detectionIndexA;
                    sparsePairCandidate.anchorCatalogIndex = brightCatalogStars[firstCatalog].catalogIndex;
                    sparsePairCandidate.secondaryAnchorDetectionIndex = detectionIndexB;
                    sparsePairCandidate.secondaryAnchorCatalogIndex = brightCatalogStars[secondCatalog].catalogIndex;
                    const auto applySparsePairAnchors = [&](Evaluation& candidate) {
                        candidate.sparseGuidedPair = useNarrowGuidedPairSeeds;
                        candidate.anchored = useNarrowGuidedPairSeeds;
                        candidate.anchorDetectionIndex = detectionIndexA;
                        candidate.anchorCatalogIndex = brightCatalogStars[firstCatalog].catalogIndex;
                        candidate.secondaryAnchorDetectionIndex = detectionIndexB;
                        candidate.secondaryAnchorCatalogIndex = brightCatalogStars[secondCatalog].catalogIndex;
                    };
                    if (context.isAcceptableSparseGuidedPairEvaluation(
                            settings,
                            catalogContext,
                            starDetections,
                            sparsePairCandidate))
                    {
                        appendDetectionPairSeed(sparsePairCandidate, priorityCatalogPair);
                        if (detectionPairHardStop()) {
                            break;
                        }
                    }

                    const QVector<int>& verificationDetectionIndices = useNarrowGuidedPairSeeds
                        ? *pairSeedVerificationDetectionIndices
                        : seedMatchDetectionIndices;
                    const Evaluation candidate = context.evaluatePose(
                        settings,
                        catalogContext,
                        imageSize,
                        captureDateTimeUtc,
                        starDetections,
                        verificationDetectionIndices,
                        seededCandidate.azimuthDegrees,
                        seededCandidate.elevationDegrees,
                        seededCandidate.rollDegrees,
                        seededCandidate.fovDegrees,
                        nullptr,
                        fixedCenterOffsetX,
                        fixedCenterOffsetY,
                        fixedDistortionK1);
                    const Evaluation verifiedCandidate = context.verifyBlindSeedCandidate(
                        settings,
                        catalogContext,
                        imageSize,
                        captureDateTimeUtc,
                        starDetections,
                        verificationDetectionIndices,
                        candidate);
                    if (verifiedCandidate.valid)
                    {
                        Evaluation verifiedSparsePairCandidate = verifiedCandidate;
                        applySparsePairAnchors(verifiedSparsePairCandidate);
                        appendDetectionPairSeed(verifiedSparsePairCandidate, priorityCatalogPair);
                    }
                }
            }

        result.cancelled = context.isCancellationRequested();
        return result;
    };

    const int brightPairSeedWorkerThreadCount = [&]() {
        if (!useNarrowGuidedPairSeeds || (brightDetectionIndices.size() < 64)) {
            return 1;
        }
        const int idealThreadCount = QThread::idealThreadCount();
        if (idealThreadCount <= 2) {
            return 1;
        }
        const int workLimitedThreadCount = brightDetectionIndices.size() / 12;
        return std::max(1, std::min(workLimitedThreadCount, idealThreadCount - 2));
    }();
    recordProfileMetric(QStringLiteral("search.brightPairSeedThreads"), brightPairSeedWorkerThreadCount);

    const auto mergeBrightPairSeedWorkResult = [&](const BrightPairSeedWorkResult& result) {
        seedEvaluations += result.seedEvaluations;
        if (result.cancelled) {
            cancelledSeedSearch = true;
            return;
        }
        for (const Evaluation& seed : result.seeds)
        {
            if (shouldStopBrightPairSeedSearch()) {
                break;
            }
            appendVerifiedSeed(seed);
        }
    };
    for (double seedFov : seedFovs)
    {
        if (shouldStopBrightPairSeedSearch()) break;
        const SkyProjector baseProjector = createProjector(
            settings,
            imageSize,
            0.0,
            90.0,
            0.0,
            seedFov,
            fixedCenterOffsetX,
            fixedCenterOffsetY,
            fixedDistortionK1);
        if (!baseProjector.valid) {
            continue;
        }
        QVector<SkyVector> brightDetectionVectors(brightDetectionIndices.size());
        QVector<quint8> brightDetectionVectorValid(brightDetectionIndices.size(), 0);
        for (int brightDetection = 0; brightDetection < brightDetectionIndices.size(); ++brightDetection)
        {
            const int detectionIndex = brightDetectionIndices[brightDetection];
            if ((detectionIndex >= 0)
                && (detectionIndex < starDetections.size())
                && unprojectPixelToVector(baseProjector, starDetections[detectionIndex].m_center, brightDetectionVectors[brightDetection]))
            {
                brightDetectionVectorValid[brightDetection] = 1;
            }
        }
        if (brightPairSeedWorkerThreadCount <= 1)
        {
            for (int firstDetection = 0; firstDetection < brightDetectionIndices.size(); ++firstDetection)
            {
                if (shouldStopBrightPairSeedSearch()) {
                    break;
                }
                const BrightPairSeedWorkResult workResult = processBrightPairFirstDetection(
                    *this,
                    baseProjector,
                    brightDetectionVectors,
                    brightDetectionVectorValid,
                    seedFov,
                    firstDetection);
                mergeBrightPairSeedWorkResult(workResult);
            }
        }
        else
        {
            QThreadPool brightPairSeedPool;
            brightPairSeedPool.setMaxThreadCount(brightPairSeedWorkerThreadCount);
            for (int batchStart = 0; batchStart < brightDetectionIndices.size(); batchStart += brightPairSeedWorkerThreadCount)
            {
                if (shouldStopBrightPairSeedSearch()) {
                    break;
                }
                const int remainingDetectionCount = static_cast<int>(brightDetectionIndices.size()) - batchStart;
                const int batchCount = std::min(brightPairSeedWorkerThreadCount, remainingDetectionCount);
                QVector<BrightPairSeedWorkResult> batchResults(batchCount);
                for (int batchIndex = 0; batchIndex < batchCount; ++batchIndex)
                {
                    const int firstDetection = batchStart + batchIndex;
                    brightPairSeedPool.start(QRunnable::create([&, batchIndex, firstDetection, seedFov, baseProjector]() {
                        SolverContext workerContext(m_owner);
                        workerContext.copySearchStateFrom(*this);
                        batchResults[batchIndex] = processBrightPairFirstDetection(
                            workerContext,
                            baseProjector,
                            brightDetectionVectors,
                            brightDetectionVectorValid,
                            seedFov,
                            firstDetection);
                    }));
                }
                brightPairSeedPool.waitForDone();
                for (const BrightPairSeedWorkResult& workResult : batchResults)
                {
                    mergeBrightPairSeedWorkResult(workResult);
                    if (shouldStopBrightPairSeedSearch()) {
                        break;
                    }
                }
            }
        }
    }
    recordProfileMetric(QStringLiteral("search.brightPairSeedEvaluations"), seedEvaluations);
    recordProfileMetric(QStringLiteral("search.brightPairVerifiedSeeds"), verifiedSeeds);

    QVector<Evaluation> sparseGuidedPairSeeds;
    if (useNarrowGuidedPairSeeds)
    {
        for (const Evaluation& seed : seeds)
        {
            if (seed.sparseGuidedPair) {
                sparseGuidedPairSeeds.append(seed);
            }
        }
        std::sort(sparseGuidedPairSeeds.begin(), sparseGuidedPairSeeds.end(), [this](const Evaluation& lhs, const Evaluation& rhs) {
            return isBetterGuidedDirectionEvaluation(lhs, rhs);
        });
        if (qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE"))
        {
            const int debugSeedCount = std::min(64, static_cast<int>(sparseGuidedPairSeeds.size()));
            qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: sparse guided pair seeds"
                     << sparseGuidedPairSeeds.size()
                     << "showing" << debugSeedCount;
            for (int i = 0; i < debugSeedCount; ++i)
            {
                const Evaluation& seed = sparseGuidedPairSeeds[i];
                const QString primaryName = ((seed.anchorCatalogIndex >= 0) && (seed.anchorCatalogIndex < catalogContext.catalogStars.size()))
                    ? catalogDisplayName(catalogContext.catalogStars[seed.anchorCatalogIndex])
                    : QString();
                const QString secondaryName = ((seed.secondaryAnchorCatalogIndex >= 0) && (seed.secondaryAnchorCatalogIndex < catalogContext.catalogStars.size()))
                    ? catalogDisplayName(catalogContext.catalogStars[seed.secondaryAnchorCatalogIndex])
                    : QString();
                qCDebug(cameraPlateSolverLog).noquote().nospace()
                    << "CameraPlateSolver: sparse seed #" << i
                    << " Az=" << seed.azimuthDegrees
                    << " El=" << seed.elevationDegrees
                    << " Roll=" << seed.rollDegrees
                    << " FoV=" << seed.fovDegrees
                    << " matches=" << seed.matchCount
                    << " RMS=" << seed.rmsErrorPixels
                    << " primary=" << primaryName << "/" << seed.anchorDetectionIndex
                    << " secondary=" << secondaryName << "/" << seed.secondaryAnchorDetectionIndex;
            }
        }
        if (sparseGuidedPairSeeds.size() > 256)
        {
            QVector<Evaluation> retainedSparseSeeds;
            retainedSparseSeeds.reserve(512);
            QSet<qint64> retainedPoseCells;
            const double cellSizeDegrees = std::max(0.4, std::min(1.5, static_cast<double>(settings.m_fov)));
            const double rollCellSizeDegrees = std::max(10.0, std::min(45.0, static_cast<double>(settings.m_fov) * 12.0));
            const auto poseCellKey = [cellSizeDegrees, rollCellSizeDegrees](const Evaluation& seed) {
                const qint64 azCell = static_cast<qint64>(std::floor(normalizeDegrees(seed.azimuthDegrees) / cellSizeDegrees));
                const qint64 elCell = static_cast<qint64>(std::floor((std::clamp(seed.elevationDegrees, -90.0, 90.0) + 90.0) / cellSizeDegrees));
                const qint64 rollCell = static_cast<qint64>(std::floor(normalizeDegrees(seed.rollDegrees) / rollCellSizeDegrees));
                return ((azCell & 0xfffffLL) << 40)
                    ^ ((elCell & 0xfffffLL) << 12)
                    ^ (rollCell & 0xfffLL);
            };
            const auto appendSparseSeed = [&](const Evaluation& seed) {
                const bool alreadyKept = std::any_of(retainedSparseSeeds.cbegin(), retainedSparseSeeds.cend(), [&seed](const Evaluation& retainedSeed) {
                    return sameEvaluationIdentity(retainedSeed, seed);
                });
                if (alreadyKept) {
                    return false;
                }
                retainedSparseSeeds.append(seed);
                retainedPoseCells.insert(poseCellKey(seed));
                return true;
            };
            for (int i = 0; (i < sparseGuidedPairSeeds.size()) && (retainedSparseSeeds.size() < 256); ++i) {
                appendSparseSeed(sparseGuidedPairSeeds[i]);
            }

            QVector<Evaluation> diverseSparseSeeds = sparseGuidedPairSeeds;
            std::sort(diverseSparseSeeds.begin(), diverseSparseSeeds.end(), [this](const Evaluation& lhs, const Evaluation& rhs) {
                return isBetterWeakModeEvaluation(lhs, rhs);
            });
            for (const Evaluation& seed : diverseSparseSeeds)
            {
                if (retainedSparseSeeds.size() >= 512) {
                    break;
                }
                const qint64 cellKey = poseCellKey(seed);
                if (retainedPoseCells.contains(cellKey)) {
                    continue;
                }
                appendSparseSeed(seed);
            }
            sparseGuidedPairSeeds = std::move(retainedSparseSeeds);
        }
    }

    std::sort(seeds.begin(), seeds.end(), [this](const Evaluation& lhs, const Evaluation& rhs) {
        return isBetterWeakModeEvaluation(lhs, rhs);
    });
    if (seeds.size() > 64) {
        seeds.resize(64);
    }
    for (const Evaluation& sparseSeed : sparseGuidedPairSeeds)
    {
        const bool alreadyKept = std::any_of(seeds.cbegin(), seeds.cend(), [&sparseSeed](const Evaluation& seed) {
            return sameEvaluationIdentity(seed, sparseSeed);
        });
        if (!alreadyKept) {
            seeds.append(sparseSeed);
        }
    }

    return seeds;
}

QVector<CameraPlateSolver::SolverContext::Evaluation> CameraPlateSolver::SolverContext::buildVectorQuadBlindSeeds(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const QVector<VisibleCatalogStar>& visibleStars, double seedFovOverride)
{
    QVector<Evaluation> seeds;
    if (isCancellationRequested() || (visibleStars.size() < settings.m_plateSolveMinMatches)) {
        return seeds;
    }

    // Mode 0 (blind FoV) is supported via an explicit seedFovOverride: the caller
    // sweeps a set of candidate FoVs, calling this once per FoV. Without an
    // override, mode 0 falls through to the wide-fallback grid below (this engine
    // needs an assumed FoV to unproject pixels to camera-frame rays).
    if (!plateSolveStartUsesFov(settings) && (seedFovOverride <= 0.0)) {
        return seeds;
    }

    const bool isWideFisheyeLens = isWidePlateSolveContext(settings);
    const QVector<int> signatureDetectionIndices = isWideFisheyeLens
        ? selectDetectionIndicesForBlindSignatures(starDetections, detectionIndices, 10, 10, 16)
        : detectionIndices;

    const bool useStartLens = plateSolveStartUsesLens(settings);
    const double fixedCenterOffsetX = useStartLens ? settings.m_lensCenterOffsetX : 0.0;
    const double fixedCenterOffsetY = useStartLens ? settings.m_lensCenterOffsetY : 0.0;
    const double fixedDistortionK1 = useStartLens ? settings.m_lensDistortionK1 : 0.0;
    const double seedFov = (seedFovOverride > 0.0)
        ? seedFovOverride
        : static_cast<double>(settings.m_fov);

    QVector<SkyVector> detectionRayVectors;
    QVector<quint8> detectionRayVectorValid;
    const SkyProjector referenceProjector = buildDetectionRayVectors(
        settings,
        imageSize,
        seedFov,
        fixedCenterOffsetX,
        fixedCenterOffsetY,
        fixedDistortionK1,
        starDetections,
        signatureDetectionIndices,
        detectionRayVectors,
        detectionRayVectorValid);
    if (!referenceProjector.valid || isCancellationRequested()) {
        return seeds;
    }

    // Bright-pool quad support (default): both the catalog index and the detection
    // quads are full C(k,4) over the brightest stars (catalog 24, detection ~16),
    // mirroring the proven legacy buildCatalogQuadSignatures support. The older
    // anchor+nearest-neighbour structure produced 0 code matches on wide fisheye
    // mode-1 images (e.g. stars-wide-2.jpg row 15) because the catalog drew
    // neighbours from all visible stars while detections drew from only the
    // brightest ~16, so a bright star's nearest catalog neighbours (faint nearby
    // stars) never matched its nearest detected neighbours. The bright-pool fix
    // takes stars-wide-2.jpg row 15 from solved=false to a clean solve and leaves
    // the synthetic fisheye mode-1 corpus unchanged. The old path is kept behind
    // an env var for A/B comparison.
    const bool useBrightPoolQuads = !qEnvironmentVariableIsSet(
        "SDRANGEL_CAMERA_PLATE_SOLVER_QUAD_ANCHOR_NEIGHBOUR_POOL");
    constexpr int catalogBrightPoolLimit = 24;

    const CatalogQuadCodeIndex catalogIndex = buildCatalogQuadCodeIndex(
        visibleStars, true, useBrightPoolQuads ? catalogBrightPoolLimit : 0);
    if (isCancellationRequested() || catalogIndex.entries.isEmpty()) {
        return seeds;
    }
    recordProfileMetric(QStringLiteral("search.vectorQuadCatalogEntries"), catalogIndex.entries.size());

    const int maxDetectionCount = std::min<int>(
        isWideFisheyeLens ? 16 : 14,
        static_cast<int>(detectionRayVectors.size()));
    constexpr double codeEpsilon = 0.02;
    constexpr int maxVerified = 50;

    const int minBlindSeedMatches = std::max(
        settings.m_plateSolveMinMatches + 1,
        std::min(6, static_cast<int>(detectionIndices.size())));

    struct QuadHypothesis
    {
        double score = std::numeric_limits<double>::infinity();
        std::array<int, 4> detectionIndicesIdx {{-1, -1, -1, -1}}; // indices into detectionRayVectors, canonical A,B,C,D
        std::array<int, 4> catalogVisibleIndices {{-1, -1, -1, -1}}; // indices into visibleStars, canonical A,B,C,D
    };
    QVector<QuadHypothesis> hypotheses;

    qint64 detectionQuadsConsidered = 0;
    qint64 codeMatches = 0;

    // Process one detection quad (4 indices into detectionRayVectors): compute its
    // vector code, query the catalog index epsilon-ball, and append a hypothesis per
    // catalog match. Returns true if the quad produced a valid code (was considered).
    const auto processDetectionQuad = [&](const std::array<int, 4>& idx) -> bool {
        const std::array<SkyVector, 4> vectors {{
            detectionRayVectors[idx[0]],
            detectionRayVectors[idx[1]],
            detectionRayVectors[idx[2]],
            detectionRayVectors[idx[3]]
        }};
        const QuadVectorCode detectionCode = buildVectorQuadCode(vectors);
        if (!detectionCode.valid) {
            return false;
        }
        ++detectionQuadsConsidered;

        const double target[5] = {
            detectionCode.xC,
            detectionCode.yC,
            detectionCode.xD,
            detectionCode.yD,
            detectionCode.angleABRadians
        };
        catalogIndex.queryEpsilonBall(target, codeEpsilon, [&](int entryIndex) {
            ++codeMatches;
            const CatalogQuadCodeEntry& entry = catalogIndex.entries[entryIndex];
            double codeDistanceSquared = 0.0;
            for (int d = 0; d < catalogIndex.dimensions; ++d) {
                const double diff = entry.code[d] - target[d];
                codeDistanceSquared += diff * diff;
            }
            double brightnessSum = 0.0;
            for (int corner = 0; corner < 4; ++corner) {
                brightnessSum += visibleStars[entry.visibleStarIndices[corner]].magnitude;
            }

            QuadHypothesis hypothesis;
            hypothesis.score = codeDistanceSquared + brightnessSum * 0.01;
            for (int corner = 0; corner < 4; ++corner) {
                hypothesis.detectionIndicesIdx[corner] = idx[detectionCode.order[corner]];
                hypothesis.catalogVisibleIndices[corner] = entry.visibleStarIndices[corner];
            }
            hypotheses.append(hypothesis);
        });
        return true;
    };

    if (useBrightPoolQuads)
    {
        // Detection quads: unstructured C(k,4) over the brightest maxDetectionCount
        // detections, mirroring the bright-pool catalog index support so any 4
        // detected bright stars have their quad represented on both sides.
        for (int a = 0; (a < maxDetectionCount) && !isCancellationRequested(); ++a)
        {
            if (!detectionRayVectorValid[a]) continue;
            for (int b = a + 1; b < maxDetectionCount; ++b)
            {
                if (!detectionRayVectorValid[b]) continue;
                for (int c = b + 1; c < maxDetectionCount; ++c)
                {
                    if (!detectionRayVectorValid[c]) continue;
                    for (int d = c + 1; d < maxDetectionCount; ++d)
                    {
                        if (!detectionRayVectorValid[d]) continue;
                        processDetectionQuad(std::array<int, 4>{{a, b, c, d}});
                    }
                }
            }
        }
    }
    else
    {
        // Detection quads are generated anchor-centric (each detection + 3-of-its-
        // angularly-nearest neighbours), mirroring buildCatalogQuadCodeIndex's
        // anchor+neighbour structure with the same band/counts.
        constexpr int detectionNeighborCount = 6;
        constexpr int maxQuadsPerDetectionAnchor = 8;
        constexpr double minDetectionNeighborSeparationDegrees = 2.0;
        constexpr double maxDetectionNeighborSeparationDegrees = 70.0;
        const double minDetectionNeighborDot = std::cos(degToRad(maxDetectionNeighborSeparationDegrees));
        const double maxDetectionNeighborDot = std::cos(degToRad(minDetectionNeighborSeparationDegrees));

        for (int anchor = 0; (anchor < maxDetectionCount) && !isCancellationRequested(); ++anchor)
        {
            if (!detectionRayVectorValid[anchor]) continue;

            QVector<std::pair<double, int>> neighborDistances;
            neighborDistances.reserve(maxDetectionCount);
            for (int j = 0; j < maxDetectionCount; ++j)
            {
                if ((j == anchor) || !detectionRayVectorValid[j]) continue;
                const double angleDot = std::clamp(dot(detectionRayVectors[anchor], detectionRayVectors[j]), -1.0, 1.0);
                if ((angleDot < minDetectionNeighborDot) || (angleDot > maxDetectionNeighborDot)) {
                    continue;
                }
                neighborDistances.append({std::acos(angleDot), j});
            }

            const int kCount = std::min<int>(detectionNeighborCount, static_cast<int>(neighborDistances.size()));
            if (kCount < 3) continue;
            std::partial_sort(neighborDistances.begin(), neighborDistances.begin() + kCount, neighborDistances.end(),
                [](const std::pair<double, int>& lhs, const std::pair<double, int>& rhs) {
                    return lhs.first < rhs.first;
                });

            int quadsForAnchor = 0;
            for (int a = 0; (a < kCount) && (quadsForAnchor < maxQuadsPerDetectionAnchor); ++a)
            {
                for (int b = a + 1; (b < kCount) && (quadsForAnchor < maxQuadsPerDetectionAnchor); ++b)
                {
                    for (int c = b + 1; (c < kCount) && (quadsForAnchor < maxQuadsPerDetectionAnchor); ++c)
                    {
                        const std::array<int, 4> idx {{
                            anchor,
                            neighborDistances[a].second,
                            neighborDistances[b].second,
                            neighborDistances[c].second
                        }};
                        if (processDetectionQuad(idx)) {
                            ++quadsForAnchor;
                        }
                    }
                }
            }
        }
    }
    recordProfileMetric(QStringLiteral("search.vectorQuadDetectionQuads"), detectionQuadsConsidered);
    recordProfileMetric(QStringLiteral("search.vectorQuadCodeMatches"), codeMatches);
    if (isCancellationRequested() || hypotheses.isEmpty()) {
        return seeds;
    }

    std::sort(hypotheses.begin(), hypotheses.end(), [](const QuadHypothesis& lhs, const QuadHypothesis& rhs) {
        return lhs.score < rhs.score;
    });

    bool earlyExit = false;
    qint64 verifiedSeeds = 0;
    const int verifyLimit = std::min<int>(maxVerified, static_cast<int>(hypotheses.size()));
    for (int h = 0; h < verifyLimit; ++h)
    {
        if (earlyExit || isCancellationRequested()) break;
        const QuadHypothesis& hypothesis = hypotheses[h];

        QVector<SkyVector> sourceVectors(4);
        QVector<SkyVector> targetVectors(4);
        QVector<int> allowedCatalogIndices;
        allowedCatalogIndices.reserve(4);
        for (int corner = 0; corner < 4; ++corner) {
            sourceVectors[corner] = detectionRayVectors[hypothesis.detectionIndicesIdx[corner]];
            targetVectors[corner] = visibleStars[hypothesis.catalogVisibleIndices[corner]].vector;
            allowedCatalogIndices.append(visibleStars[hypothesis.catalogVisibleIndices[corner]].catalogIndex);
        }

        double azimuthDegrees = 0.0;
        double elevationDegrees = 0.0;
        double rollDegrees = 0.0;
        if (!poseFromVectorPairsN(referenceProjector, sourceVectors, targetVectors, azimuthDegrees, elevationDegrees, rollDegrees)) {
            continue;
        }

        const Evaluation seededCandidate = evaluatePose(
            settings,
            catalogContext,
            imageSize,
            captureDateTimeUtc,
            starDetections,
            detectionIndices,
            azimuthDegrees,
            elevationDegrees,
            rollDegrees,
            seedFov,
            &allowedCatalogIndices,
            fixedCenterOffsetX,
            fixedCenterOffsetY,
            fixedDistortionK1);
        ++verifiedSeeds;
        const bool debugSparseQuad = qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE");
        if (!seededCandidate.valid) {
            if (debugSparseQuad) {
                qCDebug(cameraPlateSolverLog).noquote().nospace()
                    << "CameraPlateSolver[vector-quad-hyp] h=" << h
                    << " score=" << hypothesis.score
                    << " poseAz=" << azimuthDegrees << " poseEl=" << elevationDegrees << " poseRoll=" << rollDegrees
                    << " seededCandidate.valid=false";
            }
            continue;
        }

        const Evaluation candidate = evaluatePose(
            settings,
            catalogContext,
            imageSize,
            captureDateTimeUtc,
            starDetections,
            detectionIndices,
            seededCandidate.azimuthDegrees,
            seededCandidate.elevationDegrees,
            seededCandidate.rollDegrees,
            seededCandidate.fovDegrees,
            nullptr,
            fixedCenterOffsetX,
            fixedCenterOffsetY,
            fixedDistortionK1);

        const Evaluation verifiedCandidate = verifyBlindSeedCandidate(
            settings,
            catalogContext,
            imageSize,
            captureDateTimeUtc,
            starDetections,
            detectionIndices,
            candidate);
        if (debugSparseQuad) {
            // Note: keep each qCDebug(cameraPlateSolverLog) statement to <=9 QString::arg() placeholders -
            // 10+ placeholders (i.e. reaching %10/%11) truncates the output on this toolchain.
            qCDebug(cameraPlateSolverLog).noquote() << QStringLiteral("CameraPlateSolver[vector-quad-hyp] h=%1 seeded.matches=%2 seeded.rms=%3")
                .arg(h).arg(seededCandidate.matchCount).arg(seededCandidate.rmsErrorPixels);
            qCDebug(cameraPlateSolverLog).noquote() << QStringLiteral("CameraPlateSolver[vector-quad-hyp] h=%1 candidate.matches=%2 candidate.rms=%3 candidate.valid=%4 verified.valid=%5")
                .arg(h).arg(candidate.matchCount).arg(candidate.rmsErrorPixels).arg(candidate.valid).arg(verifiedCandidate.valid);
        }
        if (verifiedCandidate.valid) {
            seeds.append(verifiedCandidate);
            if (debugSparseQuad)
            {
                qCDebug(cameraPlateSolverLog).noquote().nospace()
                    << "CameraPlateSolver[vector-quad-seed] h=" << h
                    << " score=" << hypothesis.score
                    << " Az=" << verifiedCandidate.azimuthDegrees
                    << " El=" << verifiedCandidate.elevationDegrees
                    << " Roll=" << verifiedCandidate.rollDegrees
                    << " FoV=" << verifiedCandidate.fovDegrees
                    << " matches=" << verifiedCandidate.matchCount
                    << " RMS=" << verifiedCandidate.rmsErrorPixels;
            }
            if (verifiedCandidate.matchCount >= minBlindSeedMatches + 3
                && verifiedCandidate.rmsErrorPixels < kBlindSeedMaxRmsPixels * 0.5)
            {
                earlyExit = true;
            }
        } else {
            // The full re-evaluation from a quad-derived pose often lands with many
            // matches but a coarser RMS than verifyBlindSeedCandidate's tight gate
            // (tuned for already-refined poses). Accept it directly with the same
            // relaxed cap used for guided-triangle seeds, so the final solver gets
            // a chance to refine it further.
            const double relaxedSeedRmsCap = std::min(
                std::max(18.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.75),
                static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.90);
            if (candidate.valid
                && (candidate.matchCount >= minBlindSeedMatches)
                && (candidate.rmsErrorPixels <= relaxedSeedRmsCap)
                && hasAcceptableBrightnessConsistency(candidate))
            {
                seeds.append(candidate);
                if (debugSparseQuad) {
                    qCDebug(cameraPlateSolverLog).noquote() << QStringLiteral("CameraPlateSolver[vector-quad-seed-relaxed] h=%1 matches=%2 rms=%3")
                        .arg(h).arg(candidate.matchCount).arg(candidate.rmsErrorPixels);
                }
            }
        }
    }
    recordProfileMetric(QStringLiteral("search.vectorQuadVerifiedSeeds"), verifiedSeeds);

    std::sort(seeds.begin(), seeds.end(), [this](const Evaluation& lhs, const Evaluation& rhs) {
        return isBetterWeakModeEvaluation(lhs, rhs);
    });
    const int seedLimit = isWideFisheyeLens ? 64 : 12;
    return selectConsensusSeedRepresentatives(settings, seeds, seedLimit, "vector-quad");
}

const QVector<double>& CameraPlateSolver::SolverContext::detectionBrightnessRanks(const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices)
{
    if (m_detectionBrightnessRankCache.size() != starDetections.size())
    {
        m_detectionBrightnessRankCache.resize(starDetections.size());
        std::fill(m_detectionBrightnessRankCache.begin(), m_detectionBrightnessRankCache.end(), 0.5);
        m_detectionBrightnessRankIndices.clear();
    }
    if (m_detectionBrightnessRankIndices == detectionIndices) {
        return m_detectionBrightnessRankCache;
    }

    for (int detectionIndex : m_detectionBrightnessRankIndices)
    {
        if ((detectionIndex >= 0) && (detectionIndex < m_detectionBrightnessRankCache.size())) {
            m_detectionBrightnessRankCache[detectionIndex] = 0.5;
        }
    }

    QVector<int> sorted = detectionIndices;
    std::sort(sorted.begin(), sorted.end(), [this, &starDetections](int lhs, int rhs) {
        const double lhsBrightness = cachedDetectionBrightnessMetric(starDetections, lhs);
        const double rhsBrightness = cachedDetectionBrightnessMetric(starDetections, rhs);
        if (!qFuzzyCompare(lhsBrightness + 1.0, rhsBrightness + 1.0)) {
            return lhsBrightness > rhsBrightness;
        }
        return starDetections[lhs].m_qualityScore > starDetections[rhs].m_qualityScore;
    });

    const double divisor = std::max(1, static_cast<int>(sorted.size()) - 1);
    for (int i = 0; i < sorted.size(); ++i)
    {
        const int detectionIndex = sorted[i];
        if ((detectionIndex >= 0) && (detectionIndex < m_detectionBrightnessRankCache.size())) {
            m_detectionBrightnessRankCache[detectionIndex] = static_cast<double>(i) / divisor;
        }
    }
    m_detectionBrightnessRankIndices = detectionIndices;
    return m_detectionBrightnessRankCache;
}

const QVector<double>& CameraPlateSolver::SolverContext::projectedBrightnessRanks(const QVector<ProjectedCatalogStar>& projectedStars)
{
    m_projectedBrightnessRankScratch.resize(projectedStars.size());
    bool alreadyMagnitudeSorted = true;
    for (int i = 1; i < projectedStars.size(); ++i)
    {
        if (projectedStars[i - 1].magnitude > projectedStars[i].magnitude)
        {
            alreadyMagnitudeSorted = false;
            break;
        }
    }
    if (alreadyMagnitudeSorted)
    {
        const double divisor = std::max(1, static_cast<int>(projectedStars.size()) - 1);
        for (int i = 0; i < projectedStars.size(); ++i) {
            m_projectedBrightnessRankScratch[i] = static_cast<double>(i) / divisor;
        }
        return m_projectedBrightnessRankScratch;
    }

    QVector<int>& sorted = m_projectedBrightnessRankSortScratch;
    sorted.clear();
    sorted.reserve(projectedStars.size());
    for (int i = 0; i < projectedStars.size(); ++i) {
        sorted.append(i);
    }
    std::sort(sorted.begin(), sorted.end(), [&projectedStars](int lhs, int rhs) {
        return projectedStars[lhs].magnitude < projectedStars[rhs].magnitude;
    });

    const double divisor = std::max(1, static_cast<int>(sorted.size()) - 1);
    for (int i = 0; i < sorted.size(); ++i) {
        m_projectedBrightnessRankScratch[sorted[i]] = static_cast<double>(i) / divisor;
    }
    return m_projectedBrightnessRankScratch;
}

double CameraPlateSolver::SolverContext::matchBrightnessRankError(const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const QVector<ProjectedCatalogStar>& projectedStars, const QVector<Match>& matches)
{
    if (matches.size() < 2) {
        return 0.0;
    }

    const QVector<double>& detectionRanks = detectionBrightnessRanks(starDetections, detectionIndices);
    const QVector<double>& projectedRanks = projectedBrightnessRanks(projectedStars);
    int maxCatalogIndex = -1;
    for (const ProjectedCatalogStar& projectedStar : projectedStars) {
        maxCatalogIndex = std::max(maxCatalogIndex, projectedStar.catalogIndex);
    }
    if (maxCatalogIndex >= 0)
    {
        const int oldSize = m_catalogBrightnessRankGeneration.size();
        if (oldSize <= maxCatalogIndex)
        {
            m_catalogBrightnessRankGeneration.resize(maxCatalogIndex + 1);
            std::fill(
                m_catalogBrightnessRankGeneration.begin() + oldSize,
                m_catalogBrightnessRankGeneration.end(),
                0);
            m_catalogBrightnessRankScratch.resize(maxCatalogIndex + 1);
        }
    }
    {
        const int oldSize = m_brightnessMatchedDetectionGeneration.size();
        if (oldSize < starDetections.size())
        {
            m_brightnessMatchedDetectionGeneration.resize(starDetections.size());
            std::fill(
                m_brightnessMatchedDetectionGeneration.begin() + oldSize,
                m_brightnessMatchedDetectionGeneration.end(),
                0);
        }
    }
    ++m_brightnessRankGeneration;
    if (m_brightnessRankGeneration == std::numeric_limits<int>::max())
    {
        std::fill(m_catalogBrightnessRankGeneration.begin(), m_catalogBrightnessRankGeneration.end(), 0);
        std::fill(m_brightnessMatchedDetectionGeneration.begin(), m_brightnessMatchedDetectionGeneration.end(), 0);
        m_brightnessRankGeneration = 1;
    }
    const int rankGeneration = m_brightnessRankGeneration;
    for (int i = 0; i < projectedStars.size(); ++i)
    {
        const int catalogIndex = projectedStars[i].catalogIndex;
        if ((catalogIndex >= 0) && (catalogIndex < m_catalogBrightnessRankGeneration.size()))
        {
            m_catalogBrightnessRankGeneration[catalogIndex] = rankGeneration;
            m_catalogBrightnessRankScratch[catalogIndex] = (i < projectedRanks.size()) ? projectedRanks[i] : 0.5;
        }
    }

    double sumError = 0.0;
    int count = 0;
    for (const Match& match : matches)
    {
        if ((match.detectionIndex >= 0) && (match.detectionIndex < m_brightnessMatchedDetectionGeneration.size())) {
            m_brightnessMatchedDetectionGeneration[match.detectionIndex] = rankGeneration;
        }
        const double detectionRank = ((match.detectionIndex >= 0) && (match.detectionIndex < detectionRanks.size()))
            ? detectionRanks[match.detectionIndex]
            : 0.5;
        if ((match.catalogIndex < 0)
            || (match.catalogIndex >= m_catalogBrightnessRankGeneration.size())
            || (m_catalogBrightnessRankGeneration[match.catalogIndex] != rankGeneration))
        {
            continue;
        }
        sumError += std::fabs(detectionRank - m_catalogBrightnessRankScratch[match.catalogIndex]);
        ++count;
    }

    const int brightDetectionCount = std::min(
        static_cast<int>(detectionIndices.size()),
        std::max(3, std::min(8, static_cast<int>(matches.size()) + 1)));
    const double brightRankThreshold = (detectionIndices.size() > 1)
        ? static_cast<double>(brightDetectionCount - 1) / static_cast<double>(detectionIndices.size() - 1)
        : 0.0;
    for (int detectionIndex : detectionIndices)
    {
        const double detectionRank = ((detectionIndex >= 0) && (detectionIndex < detectionRanks.size()))
            ? detectionRanks[detectionIndex]
            : 0.5;
        if ((detectionRank > brightRankThreshold)
            || ((detectionIndex >= 0)
                && (detectionIndex < m_brightnessMatchedDetectionGeneration.size())
                && (m_brightnessMatchedDetectionGeneration[detectionIndex] == rankGeneration)))
        {
            continue;
        }

        sumError += 0.75 + 0.25 * (1.0 - detectionRank);
        ++count;
    }

    return count > 0 ? sumError / count : 0.0;
}

void CameraPlateSolver::SolverContext::populatePoseScoringMetrics(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const QVector<ProjectedCatalogStar>& projectedStars, const QVector<CatalogStar>& catalogStars, Evaluation& evaluation, bool forceSparseSeedMetrics)
{
    if (!shouldPopulatePoseScoringMetrics(settings, evaluation, forceSparseSeedMetrics)) {
        return;
    }

    evaluation.brightnessRankError = matchBrightnessRankError(
        starDetections,
        detectionIndices,
        projectedStars,
        evaluation.matches);
    evaluation.meanCatalogMagnitude = meanCatalogMagnitudeForMatches(
        catalogStars,
        evaluation.matches);
}

QVector<CameraPlateSolver::SolverContext::Match> CameraPlateSolver::SolverContext::buildMatches(const PlateSolveCatalogContext& catalogContext, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const QVector<ProjectedCatalogStar>& projectedStars, double matchRadiusPixels)
{
    QVector<CandidatePair>& candidatePairs = m_candidatePairScratch;
    candidatePairs.clear();
    const QVector<CatalogStar>& catalogStars = catalogContext.catalogStars;
    const QVector<double>& detectionRanks = detectionBrightnessRanks(starDetections, detectionIndices);
    const QVector<double>& detectionReliability = m_detectionReliabilityMetricCache;
    const QVector<double>& projectedRanks = projectedBrightnessRanks(projectedStars);
    const bool useNarrowGuidedBrightShapePrior =
        m_useDirectionSeedPreference && (m_directionSeedReferenceFovDegrees <= 5.0);
    const double maxDistanceSquared = matchRadiusPixels * matchRadiusPixels;
    const double cellSize = std::max(1.0, matchRadiusPixels);
    QVector<int>& projectedGridHeads = m_projectedStarGridHeadsScratch;
    QVector<int>& projectedGridNext = m_projectedStarGridNextScratch;
    QVector<int>& projectedCellX = m_projectedStarGridCellXScratch;
    QVector<int>& projectedCellY = m_projectedStarGridCellYScratch;
    projectedGridHeads.clear();
    projectedGridNext.resize(projectedStars.size());
    projectedCellX.resize(projectedStars.size());
    projectedCellY.resize(projectedStars.size());
    int minCellX = std::numeric_limits<int>::max();
    int maxCellX = std::numeric_limits<int>::min();
    int minCellY = std::numeric_limits<int>::max();
    int maxCellY = std::numeric_limits<int>::min();
    for (int projectedIndex = 0; projectedIndex < projectedStars.size(); ++projectedIndex)
    {
        const QPointF& point = projectedStars[projectedIndex].point;
        const int cellX = static_cast<int>(std::floor(point.x() / cellSize));
        const int cellY = static_cast<int>(std::floor(point.y() / cellSize));
        projectedCellX[projectedIndex] = cellX;
        projectedCellY[projectedIndex] = cellY;
        minCellX = std::min(minCellX, cellX);
        maxCellX = std::max(maxCellX, cellX);
        minCellY = std::min(minCellY, cellY);
        maxCellY = std::max(maxCellY, cellY);
    }
    if (!projectedStars.isEmpty())
    {
        const int gridWidth = maxCellX - minCellX + 1;
        const int gridHeight = maxCellY - minCellY + 1;
        projectedGridHeads.resize(gridWidth * gridHeight);
        std::fill(projectedGridHeads.begin(), projectedGridHeads.end(), -1);
        for (int projectedIndex = 0; projectedIndex < projectedStars.size(); ++projectedIndex)
        {
            const int gridIndex = (projectedCellY[projectedIndex] - minCellY) * gridWidth
                + (projectedCellX[projectedIndex] - minCellX);
            projectedGridNext[projectedIndex] = projectedGridHeads[gridIndex];
            projectedGridHeads[gridIndex] = projectedIndex;
        }
    }

    for (int detectionIndex : detectionIndices)
    {
        if ((detectionIndex < 0)
            || (detectionIndex >= starDetections.size())
            || starDetections[detectionIndex].m_hotPixelSuspect)
        {
            continue;
        }
        const CameraPipelineStarDetection& detection = starDetections[detectionIndex];
        const QPointF detectionPoint = detection.m_center;
        const double detectionRank = (detectionIndex < detectionRanks.size())
            ? detectionRanks[detectionIndex]
            : 0.5;
        const double detectionReliabilityValue = (detectionIndex < detectionReliability.size())
            ? detectionReliability[detectionIndex]
            : cachedDetectionReliabilityMetric(starDetections, detectionIndex);
        const int cellX = static_cast<int>(std::floor(detectionPoint.x() / cellSize));
        const int cellY = static_cast<int>(std::floor(detectionPoint.y() / cellSize));
        for (int dy = -1; dy <= 1; ++dy)
        {
            for (int dx = -1; dx <= 1; ++dx)
            {
                const int queryCellX = cellX + dx;
                const int queryCellY = cellY + dy;
                if (queryCellX < minCellX || queryCellX > maxCellX
                    || queryCellY < minCellY || queryCellY > maxCellY)
                {
                    continue;
                }

                const int gridIndex = (queryCellY - minCellY) * (maxCellX - minCellX + 1)
                    + (queryCellX - minCellX);
                for (int projectedIndex = projectedGridHeads[gridIndex]; projectedIndex >= 0; projectedIndex = projectedGridNext[projectedIndex])
                {
                    const ProjectedCatalogStar& projected = projectedStars[projectedIndex];
                    if (useNarrowGuidedBrightShapePrior
                        && isImplausiblyCompactBrightCatalogDetection(
                            detection,
                            projected.magnitude,
                            true))
                    {
                        continue;
                    }
                    const double deltaX = detectionPoint.x() - projected.point.x();
                    const double deltaY = detectionPoint.y() - projected.point.y();
                    const double distanceSquared = deltaX * deltaX + deltaY * deltaY;
                    if (distanceSquared > maxDistanceSquared) {
                        continue;
                    }

                    const double catalogRank = (projectedIndex < projectedRanks.size())
                        ? projectedRanks[projectedIndex]
                        : 0.5;
                    const double catalogAssignmentPenalty = faintCatalogAssignmentPenalty(projected.magnitude)
                        + brightDetectionFaintCatalogAssignmentPenalty(
                            detectionRank,
                            projected.magnitude,
                            useNarrowGuidedBrightShapePrior);
                    const double detectionReliabilityLog = std::log1p(detectionReliabilityValue);
                    candidatePairs.append({
                        detectionIndex,
                        projected.catalogIndex,
                        projectedIndex,
                        std::sqrt(distanceSquared),
                        std::fabs(detectionRank - catalogRank),
                        projected.magnitude,
                        0,
                        detectionReliabilityValue,
                        catalogAssignmentPenalty,
                        detectionReliabilityLog
                    });
                }
            }
        }
    }

    // Allow a wider candidate set for large acquisition radii so wide-field weak modes
    // do not discard the true match before geometric support is computed.
    const int maxCandidatesPerDetection = std::clamp(
        static_cast<int>(std::ceil(matchRadiusPixels / 15.0)),
        4,
        10);
    std::sort(candidatePairs.begin(), candidatePairs.end(), [matchRadiusPixels](const CandidatePair& lhs, const CandidatePair& rhs) {
        if (lhs.detectionIndex != rhs.detectionIndex) {
            return lhs.detectionIndex < rhs.detectionIndex;
        }
        const double lhsCost = lhs.distancePixels
            + matchRadiusPixels * (0.75 * lhs.brightnessRankError + lhs.catalogAssignmentPenalty)
            - std::min(matchRadiusPixels * 0.25, lhs.detectionReliabilityLog);
        const double rhsCost = rhs.distancePixels
            + matchRadiusPixels * (0.75 * rhs.brightnessRankError + rhs.catalogAssignmentPenalty)
            - std::min(matchRadiusPixels * 0.25, rhs.detectionReliabilityLog);
        return lhsCost < rhsCost;
    });
    {
        QVector<CandidatePair>& cappedPairs = m_cappedCandidatePairScratch;
        cappedPairs.clear();
        cappedPairs.reserve(candidatePairs.size());
        int lastDetectionIndex = -1;
        int countForDetection = 0;
        for (const CandidatePair& pair : candidatePairs) {
            if (pair.detectionIndex != lastDetectionIndex) {
                lastDetectionIndex = pair.detectionIndex;
                countForDetection = 0;
            }
            if (countForDetection < maxCandidatesPerDetection) {
                cappedPairs.append(pair);
                ++countForDetection;
            }
        }
        candidatePairs.swap(cappedPairs);
    }

    // Geometric-support tally. For each candidate (detection, catalog) pair we count how
    // many *other* candidate pairs agree on the inter-star distance — i.e. the detection-
    // space and catalog-space distances match within tolerance. A naive O(N^2) sweep can
    // be ~65k comparisons for N=256, and this runs inside every evaluatePose call, so we
    // also cap support accumulation per pair to avoid quadratic blow-up on dense fields.
    constexpr int kGeometricSupportCap = 8;
    for (int i = 0; i < candidatePairs.size(); ++i)
    {
        if (candidatePairs[i].geometricSupport >= kGeometricSupportCap) {
            continue;
        }
        const CandidatePair& lhs = candidatePairs[i];
        const QPointF lhsDetection = starDetections[lhs.detectionIndex].m_center;
        const QPointF& lhsCatalogPoint = projectedStars[lhs.projectedIndex].point;
        for (int j = i + 1; j < candidatePairs.size(); ++j)
        {
            CandidatePair& rhsRef = candidatePairs[j];
            if (rhsRef.geometricSupport >= kGeometricSupportCap) {
                continue;
            }
            const CandidatePair& rhs = rhsRef;
            if ((lhs.detectionIndex == rhs.detectionIndex) || (lhs.catalogIndex == rhs.catalogIndex)) {
                continue;
            }

            const double detDx = lhsDetection.x() - starDetections[rhs.detectionIndex].m_center.x();
            const double detDy = lhsDetection.y() - starDetections[rhs.detectionIndex].m_center.y();
            const double detectionDistance = std::hypot(detDx, detDy);
            const QPointF& rhsCatalogPoint = projectedStars[rhs.projectedIndex].point;
            const double catDx = lhsCatalogPoint.x() - rhsCatalogPoint.x();
            const double catDy = lhsCatalogPoint.y() - rhsCatalogPoint.y();
            const double catalogDistance = std::hypot(catDx, catDy);
            const double tolerance = std::max(2.0, 0.15 * std::max(detectionDistance, catalogDistance));
            if (std::fabs(detectionDistance - catalogDistance) <= tolerance) {
                ++candidatePairs[i].geometricSupport;
                ++rhsRef.geometricSupport;
                if (candidatePairs[i].geometricSupport >= kGeometricSupportCap) {
                    break;
                }
            }
        }
    }

    const double brightnessRankWeight = 1.25;
    const double brightnessCostWeight = 0.75;
    std::sort(candidatePairs.begin(), candidatePairs.end(), [&catalogStars, &starDetections, matchRadiusPixels,
              brightnessRankWeight, brightnessCostWeight](const CandidatePair& lhs, const CandidatePair& rhs) {
        const double lhsSupportScore = static_cast<double>(lhs.geometricSupport)
            - brightnessRankWeight * lhs.brightnessRankError
            - lhs.catalogAssignmentPenalty
            + 0.20 * lhs.detectionReliabilityLog;
        const double rhsSupportScore = static_cast<double>(rhs.geometricSupport)
            - brightnessRankWeight * rhs.brightnessRankError
            - rhs.catalogAssignmentPenalty
            + 0.20 * rhs.detectionReliabilityLog;
        if (std::fabs(lhsSupportScore - rhsSupportScore) > 0.20) {
            return lhsSupportScore > rhsSupportScore;
        }
        const double lhsCost = lhs.distancePixels
            + matchRadiusPixels * (brightnessCostWeight * lhs.brightnessRankError + lhs.catalogAssignmentPenalty)
            - std::min(matchRadiusPixels * 0.25, lhs.detectionReliabilityLog);
        const double rhsCost = rhs.distancePixels
            + matchRadiusPixels * (brightnessCostWeight * rhs.brightnessRankError + rhs.catalogAssignmentPenalty)
            - std::min(matchRadiusPixels * 0.25, rhs.detectionReliabilityLog);
        if (!qFuzzyCompare(lhsCost + 1.0, rhsCost + 1.0)) {
            return lhsCost < rhsCost;
        }
        if (!qFuzzyCompare(starDetections[lhs.detectionIndex].m_qualityScore + 1.0f, starDetections[rhs.detectionIndex].m_qualityScore + 1.0f)) {
            return starDetections[lhs.detectionIndex].m_qualityScore > starDetections[rhs.detectionIndex].m_qualityScore;
        }

        return catalogStars[lhs.catalogIndex].magnitude < catalogStars[rhs.catalogIndex].magnitude;
    });

    QVector<Match> matches;
    if (m_detectionMatchGeneration.size() < starDetections.size()) {
        m_detectionMatchGeneration.resize(starDetections.size());
    }
    if (m_catalogMatchGeneration.size() < catalogStars.size()) {
        m_catalogMatchGeneration.resize(catalogStars.size());
    }
    ++m_matchGeneration;
    if (m_matchGeneration == std::numeric_limits<int>::max())
    {
        std::fill(m_detectionMatchGeneration.begin(), m_detectionMatchGeneration.end(), 0);
        std::fill(m_catalogMatchGeneration.begin(), m_catalogMatchGeneration.end(), 0);
        m_matchGeneration = 1;
    }

    for (const CandidatePair& pair : candidatePairs)
    {
        if ((pair.detectionIndex < 0)
            || (pair.detectionIndex >= m_detectionMatchGeneration.size())
            || (pair.catalogIndex < 0)
            || (pair.catalogIndex >= m_catalogMatchGeneration.size())
            || (m_detectionMatchGeneration[pair.detectionIndex] == m_matchGeneration)
            || (m_catalogMatchGeneration[pair.catalogIndex] == m_matchGeneration))
        {
            continue;
        }

        m_detectionMatchGeneration[pair.detectionIndex] = m_matchGeneration;
        m_catalogMatchGeneration[pair.catalogIndex] = m_matchGeneration;
        matches.append({pair.detectionIndex, pair.catalogIndex, pair.distancePixels});
    }

    return matches;
}

void CameraPlateSolver::SolverContext::logUnmatchedDetections(const PlateSolveCatalogContext& catalogContext, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<ProjectedCatalogStar>& projectedStars, const QVector<Match>& matches, double matchRadiusPixels)
{
    if (!kLogPlateSolveCandidates) {
        return;
    }
    QVector<bool> detectionMatched(starDetections.size(), false);
    QHash<int, bool> catalogMatched;
    catalogMatched.reserve(matches.size());
    for (const Match& match : matches)
    {
        // Guard the QVector write against Match::detectionIndex's -1 default (consistency with
        // the other supplemental-match builders; this function is debug-only).
        if ((match.detectionIndex >= 0) && (match.detectionIndex < detectionMatched.size())) {
            detectionMatched[match.detectionIndex] = true;
        }
        catalogMatched.insert(match.catalogIndex, true);
    }

    const QVector<CatalogStar>& catalogStars = catalogContext.catalogStars;
    for (int detectionIndex = 0; detectionIndex < starDetections.size(); ++detectionIndex)
    {
        if (detectionMatched[detectionIndex]) {
            continue;
        }

        const CameraPipelineStarDetection& detection = starDetections[detectionIndex];
        double nearestAnyDistance = std::numeric_limits<double>::max();
        int nearestAnyCatalogIndex = -1;
        bool nearestAnyCatalogMatched = false;
        double nearestUnmatchedDistance = std::numeric_limits<double>::max();
        int nearestUnmatchedCatalogIndex = -1;

        for (const ProjectedCatalogStar& projected : projectedStars)
        {
            const double dx = detection.m_center.x() - projected.point.x();
            const double dy = detection.m_center.y() - projected.point.y();
            const double distancePixels = std::hypot(dx, dy);
            if (distancePixels < nearestAnyDistance)
            {
                nearestAnyDistance = distancePixels;
                nearestAnyCatalogIndex = projected.catalogIndex;
                nearestAnyCatalogMatched = catalogMatched.contains(projected.catalogIndex);
            }
            if (!catalogMatched.contains(projected.catalogIndex) && (distancePixels < nearestUnmatchedDistance))
            {
                nearestUnmatchedDistance = distancePixels;
                nearestUnmatchedCatalogIndex = projected.catalogIndex;
            }
        }

        QString reason = QStringLiteral("no nearby catalog candidate");
        if (nearestAnyCatalogIndex >= 0)
        {
            if (nearestAnyDistance <= matchRadiusPixels) {
                reason = nearestAnyCatalogMatched
                    ? QStringLiteral("closest catalog star already matched")
                    : QStringLiteral("candidate within radius was not selected");
            } else if (nearestAnyDistance <= (matchRadiusPixels * 1.5)) {
                reason = QStringLiteral("just outside match radius");
            }
        }

        const QString nearestAnyName =
            ((nearestAnyCatalogIndex >= 0) && (nearestAnyCatalogIndex < catalogStars.size()))
                ? catalogStars[nearestAnyCatalogIndex].name
                : QString();
        const QString nearestUnmatchedName =
            ((nearestUnmatchedCatalogIndex >= 0) && (nearestUnmatchedCatalogIndex < catalogStars.size()))
                ? catalogStars[nearestUnmatchedCatalogIndex].name
                : QString();

        qCDebug(cameraPlateSolverLog).noquote()
            << "CameraPlateSolver: unmatched detection"
            << "index=" << detectionIndex
            << "center=" << detection.m_center
            << "quality=" << detection.m_qualityScore
            << "peak=" << detection.m_peakValue
            << "nearestAnyDistance=" << (std::isfinite(nearestAnyDistance) ? nearestAnyDistance : -1.0)
            << "nearestAnyCatalog=" << nearestAnyCatalogIndex
            << "nearestAnyName=" << nearestAnyName
            << "nearestAnyMatched=" << nearestAnyCatalogMatched
            << "nearestUnmatchedDistance=" << (std::isfinite(nearestUnmatchedDistance) ? nearestUnmatchedDistance : -1.0)
            << "nearestUnmatchedCatalog=" << nearestUnmatchedCatalogIndex
            << "nearestUnmatchedName=" << nearestUnmatchedName
            << "reason=" << reason;
    }
}

CameraPlateSolver::SolverContext::Evaluation CameraPlateSolver::SolverContext::evaluatePose(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, double azimuthDegrees, double elevationDegrees, double rollDegrees, double fovDegrees, const QVector<int>* allowedCatalogIndices, double centerOffsetXPixels, double centerOffsetYPixels, double distortionK1, double matchRadiusOverride)
{
    Q_UNUSED(captureDateTimeUtc)

    Evaluation evaluation;
    evaluation.azimuthDegrees = normalizeDegrees(azimuthDegrees);
    evaluation.elevationDegrees = elevationDegrees;
    evaluation.rollDegrees = rollDegrees;
    evaluation.fovDegrees = fovDegrees;
    evaluation.centerOffsetXPixels = centerOffsetXPixels;
    evaluation.centerOffsetYPixels = centerOffsetYPixels;
    evaluation.distortionK1 = distortionK1;

    // Default to the acquisition radius; callers (e.g. refinePoseFromMatches' final pass)
    // can pass the tighter final-match radius so the split-radii intent ("search wide,
    // accept narrow") actually influences scoring during refinement.
    const double matchRadiusPixels = (matchRadiusOverride > 0.0)
        ? matchRadiusOverride
        : settings.m_plateSolveMatchRadius;

    const SkyProjector projector = createProjector(
        settings,
        imageSize,
        evaluation.azimuthDegrees,
        evaluation.elevationDegrees,
        evaluation.rollDegrees,
        evaluation.fovDegrees,
        evaluation.centerOffsetXPixels,
        evaluation.centerOffsetYPixels,
        evaluation.distortionK1);
    if (!projector.valid) {
        return evaluation;
    }

    buildProjectedCatalogInto(
        catalogContext,
        projector,
        matchRadiusPixels,
        allowedCatalogIndices,
        m_projectedCatalogScratch);
    const QVector<ProjectedCatalogStar>& projectedStars = m_projectedCatalogScratch;
    if (projectedStars.isEmpty()) {
        return evaluation;
    }

    evaluation.matches = buildMatches(
        catalogContext,
        starDetections,
        detectionIndices,
        projectedStars,
        matchRadiusPixels);
    evaluation.matchCount = evaluation.matches.size();
    if (evaluation.matchCount <= 0) {
        return evaluation;
    }

    double sumSquaredError = 0.0;
    for (const Match& match : evaluation.matches) {
        sumSquaredError += match.distancePixels * match.distancePixels;
    }
    evaluation.rmsErrorPixels = std::sqrt(sumSquaredError / evaluation.matchCount);
    evaluation.valid = true;
    populatePoseScoringMetrics(
        settings,
        starDetections,
        detectionIndices,
        projectedStars,
        catalogContext.catalogStars,
        evaluation,
        allowedCatalogIndices && (allowedCatalogIndices->size() <= 2));
    return evaluation;
}

void CameraPlateSolver::SolverContext::buildBlindGridCache(const PlateSolveCatalogContext& catalogContext, const SkyProjector& refProjector, const QVector<int>* allowedCatalogIndices)
{
    m_blindGridCache.clear();
    if (!refProjector.valid)
        return;

    const auto appendCachedStar = [&](const VisibleCatalogStar& vs)
    {
        QPointF pt;
        if (!projectVector(refProjector, vs.vector, pt))
            return;
        BlindGridCachedStar cs;
        cs.catalogIndex = vs.catalogIndex;
        cs.dxRef        = static_cast<float>(pt.x() - refProjector.principalPointX);
        cs.dyRef        = static_cast<float>(refProjector.principalPointY - pt.y());
        cs.magnitude    = static_cast<float>(vs.magnitude);
        m_blindGridCache.push_back(cs);
    };

    if (allowedCatalogIndices)
    {
        m_blindGridCache.reserve(allowedCatalogIndices->size());
        for (int catalogIndex : *allowedCatalogIndices)
        {
            const auto it = catalogContext.visibleStarIndexByCatalogIndex.constFind(catalogIndex);
            if (it != catalogContext.visibleStarIndexByCatalogIndex.cend()) {
                appendCachedStar(catalogContext.visibleStars[*it]);
            }
        }
    }
    else
    {
        m_blindGridCache.reserve(catalogContext.visibleStars.size());
        for (const VisibleCatalogStar& visibleStar : catalogContext.visibleStars) {
            appendCachedStar(visibleStar);
        }
    }
}

void CameraPlateSolver::SolverContext::populateBlindGridProjectedCatalog(double rollDegrees, double matchRadiusPixels, const SkyProjector& refProjector)
{
    m_blindGridProjectedScratch.clear();
    if (m_blindGridCache.isEmpty() || !refProjector.valid)
        return;
    const double rollRad = degToRad(rollDegrees);
    const double cosR    = std::cos(rollRad);
    const double sinR    = std::sin(rollRad);
    const double cx      = refProjector.principalPointX;
    const double cy      = refProjector.principalPointY;
    const QRectF expandedBounds(-matchRadiusPixels,
                                -matchRadiusPixels,
                                refProjector.width  + 2.0 * matchRadiusPixels,
                                refProjector.height + 2.0 * matchRadiusPixels);
    m_blindGridProjectedScratch.reserve(m_blindGridCache.size());
    for (const BlindGridCachedStar& cs : m_blindGridCache)
    {
        const double dxRef = static_cast<double>(cs.dxRef);
        const double dyRef = static_cast<double>(cs.dyRef);
        // 2-D rotation of the (right, up) offset vector. This matches
        // createProjector's camera-basis roll: right' = right*cos - up*sin,
        // up' = right*sin + up*cos.
        const double dx = dxRef * cosR - dyRef * sinR;
        const double dy = dxRef * sinR + dyRef * cosR;
        const QPointF pt(cx + dx, cy - dy);
        if (!expandedBounds.contains(pt))
            continue;
        m_blindGridProjectedScratch.push_back({cs.catalogIndex, pt, static_cast<double>(cs.magnitude)});
    }
}

CameraPlateSolver::SolverContext::Evaluation CameraPlateSolver::SolverContext::evaluatePoseFromPrecomputedCatalog(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, double azimuthDegrees, double elevationDegrees, double rollDegrees, double fovDegrees, double centerOffsetXPixels, double centerOffsetYPixels, double distortionK1, double matchRadiusPixels)
{
    Evaluation evaluation;
    evaluation.azimuthDegrees      = normalizeDegrees(azimuthDegrees);
    evaluation.elevationDegrees    = elevationDegrees;
    evaluation.rollDegrees         = rollDegrees;
    evaluation.fovDegrees          = fovDegrees;
    evaluation.centerOffsetXPixels = centerOffsetXPixels;
    evaluation.centerOffsetYPixels = centerOffsetYPixels;
    evaluation.distortionK1        = distortionK1;

    const QVector<ProjectedCatalogStar>& projectedStars = m_blindGridProjectedScratch;
    if (projectedStars.isEmpty())
        return evaluation;

    evaluation.matches = buildMatches(
        catalogContext, starDetections, detectionIndices, projectedStars, matchRadiusPixels);
    evaluation.matchCount = evaluation.matches.size();
    if (evaluation.matchCount <= 0)
        return evaluation;

    double sumSq = 0.0;
    for (const Match& m : evaluation.matches)
        sumSq += m.distancePixels * m.distancePixels;
    evaluation.rmsErrorPixels = std::sqrt(sumSq / evaluation.matchCount);
    evaluation.valid = true;
    populatePoseScoringMetrics(
        settings,
        starDetections,
        detectionIndices,
        projectedStars,
        catalogContext.catalogStars,
        evaluation);
    return evaluation;
}

CameraPlateSolver::SolverContext::Evaluation CameraPlateSolver::SolverContext::evaluateAnchoredPose(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const QVector<int>& allowedCatalogIndices, const GuidedAnchorPair& anchor, double azimuthDegrees, double elevationDegrees, double rollDegrees, double fovDegrees, double centerOffsetXPixels, double centerOffsetYPixels, double distortionK1, double matchRadiusPixels)
{
    Q_UNUSED(captureDateTimeUtc)

    Evaluation evaluation;
    evaluation.anchored = true;
    evaluation.anchorDetectionIndex = anchor.detectionIndex;
    evaluation.anchorCatalogIndex = anchor.catalogIndex;
    evaluation.azimuthDegrees = normalizeDegrees(azimuthDegrees);
    evaluation.elevationDegrees = elevationDegrees;
    evaluation.rollDegrees = rollDegrees;
    evaluation.fovDegrees = fovDegrees;
    evaluation.centerOffsetXPixels = centerOffsetXPixels;
    evaluation.centerOffsetYPixels = centerOffsetYPixels;
    evaluation.distortionK1 = distortionK1;

    if ((anchor.detectionIndex < 0)
        || (anchor.detectionIndex >= starDetections.size())
        || (anchor.catalogIndex < 0)
        || (anchor.catalogIndex >= catalogContext.catalogStars.size()))
    {
        return evaluation;
    }

    const SkyProjector projector = createProjector(
        settings,
        imageSize,
        evaluation.azimuthDegrees,
        evaluation.elevationDegrees,
        evaluation.rollDegrees,
        evaluation.fovDegrees,
        evaluation.centerOffsetXPixels,
        evaluation.centerOffsetYPixels,
        evaluation.distortionK1);
    if (!projector.valid) {
        return evaluation;
    }

    const auto anchorVisibleIt = catalogContext.visibleStarIndexByCatalogIndex.constFind(anchor.catalogIndex);
    if (anchorVisibleIt == catalogContext.visibleStarIndexByCatalogIndex.cend()) {
        return evaluation;
    }

    QPointF anchorProjectedPoint;
    if (!projectVector(projector, catalogContext.visibleStars[*anchorVisibleIt].vector, anchorProjectedPoint)) {
        return evaluation;
    }

    const double anchorDistance = pointDistancePixels(
        starDetections[anchor.detectionIndex].m_center,
        anchorProjectedPoint);
    if (anchorDistance > matchRadiusPixels) {
        return evaluation;
    }

    buildProjectedCatalogInto(
        catalogContext,
        projector,
        matchRadiusPixels,
        allowedCatalogIndices.isEmpty() ? nullptr : &allowedCatalogIndices,
        m_projectedCatalogScratch);
    const QVector<ProjectedCatalogStar>& projectedStars = m_projectedCatalogScratch;
    if (projectedStars.isEmpty()) {
        return evaluation;
    }

    const QVector<Match> automaticMatches = buildMatches(
        catalogContext,
        starDetections,
        detectionIndices,
        projectedStars,
        matchRadiusPixels);

    evaluation.matches.reserve(automaticMatches.size() + 1);
    evaluation.matches.append({anchor.detectionIndex, anchor.catalogIndex, anchorDistance});
    QSet<int> matchedDetections;
    QSet<int> matchedCatalogStars;
    matchedDetections.insert(anchor.detectionIndex);
    matchedCatalogStars.insert(anchor.catalogIndex);
    for (const Match& match : automaticMatches)
    {
        if (matchedDetections.contains(match.detectionIndex)
            || matchedCatalogStars.contains(match.catalogIndex))
        {
            continue;
        }
        matchedDetections.insert(match.detectionIndex);
        matchedCatalogStars.insert(match.catalogIndex);
        evaluation.matches.append(match);
    }

    evaluation.matchCount = evaluation.matches.size();
    if (evaluation.matchCount <= 0) {
        return evaluation;
    }

    double sumSquaredError = 0.0;
    for (const Match& match : evaluation.matches) {
        sumSquaredError += match.distancePixels * match.distancePixels;
    }
    evaluation.rmsErrorPixels = std::sqrt(sumSquaredError / evaluation.matchCount);
    evaluation.valid = true;
    populatePoseScoringMetrics(
        settings,
        starDetections,
        detectionIndices,
        projectedStars,
        catalogContext.catalogStars,
        evaluation);
    return evaluation;
}

bool CameraPlateSolver::SolverContext::isAcceptableSparseGuidedPairEvaluation(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QVector<CameraPipelineStarDetection>& starDetections, const Evaluation& evaluation)
{
    const bool isGuidedAnchorPose = evaluation.sparseGuidedPair || evaluation.guidedTriangle;
    if (!evaluation.valid
        || !isGuidedAnchorPose
        || !plateSolveStartUsesDirection(settings)
        || isWidePlateSolveContext(settings)
        || (!isNarrowField(settings))
        || (evaluation.matches.size() < (evaluation.guidedTriangle ? 3 : 2)))
    {
        return false;
    }

    const double maxDirectionDelta = std::max(
        1.0,
        std::min(
            static_cast<double>(settings.m_plateSolveAzElSearchRadius) + static_cast<double>(settings.m_fov) * 0.5,
            static_cast<double>(settings.m_fov) * 2.5));
    if (angularDistanceDegrees(evaluation.azimuthDegrees, settings.m_azimuth) > maxDirectionDelta) {
        return false;
    }
    if (std::fabs(evaluation.elevationDegrees - settings.m_elevation) > maxDirectionDelta) {
        return false;
    }

    const double maxFovDelta = std::max(0.08, static_cast<double>(settings.m_fov) * 0.12);
    if (std::fabs(evaluation.fovDegrees - settings.m_fov) > maxFovDelta) {
        return false;
    }
    if (plateSolveStartUsesRoll(settings)
        && (angularDistanceDegrees(evaluation.rollDegrees, settings.m_roll)
            > std::max(5.0, static_cast<double>(settings.m_fov) * 4.0)))
    {
        return false;
    }

    const double maxPairRms = sparseGuidedMaxRms(settings, evaluation.matchCount);
    if (!std::isfinite(evaluation.rmsErrorPixels) || (evaluation.rmsErrorPixels > maxPairRms)) {
        return false;
    }

    const double maxAnchorDistance = std::min(
        static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.70,
        14.0);
    int strongNamedMatches = 0;
    QSet<int> matchedDetections;
    QSet<int> matchedCatalogStars;
    for (const Match& match : evaluation.matches)
    {
        if ((match.detectionIndex < 0)
            || (match.detectionIndex >= starDetections.size())
            || (match.catalogIndex < 0)
            || (match.catalogIndex >= catalogContext.catalogStars.size())
            || matchedDetections.contains(match.detectionIndex)
            || matchedCatalogStars.contains(match.catalogIndex))
        {
            continue;
        }

        const CatalogStar& catalogStar = catalogContext.catalogStars[match.catalogIndex];
        if ((catalogStar.magnitude > std::min(static_cast<double>(settings.m_plateSolveMaxMagnitude), kNarrowGuidedBrightCatalogMaxMagnitude))
            || !isNamedSparseGuidedCatalogStar(catalogStar)
            || !isStrongSparseGuidedDetection(starDetections[match.detectionIndex])
            || (match.distancePixels > maxAnchorDistance))
        {
            continue;
        }

        matchedDetections.insert(match.detectionIndex);
        matchedCatalogStars.insert(match.catalogIndex);
        ++strongNamedMatches;
    }

    return strongNamedMatches >= (evaluation.guidedTriangle ? 3 : 2);
}

bool CameraPlateSolver::SolverContext::isAcceptableSparseGuidedPairFinalPass(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QVector<CameraPipelineStarDetection>& starDetections, const FinalMatchPassEvaluation& finalPass)
{
    const bool debugSparse = qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE");
    const bool isGuidedAnchorPose = finalPass.pose.sparseGuidedPair || finalPass.pose.guidedTriangle;
    if (!finalPass.projectorValid
        || !isGuidedAnchorPose
        || (finalPass.finalMatches.size() < std::max(4, settings.m_plateSolveMinMatches)))
    {
        if (debugSparse && finalPass.projectorValid && isGuidedAnchorPose)
        {
            qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: sparse final rejected before evaluation"
                     << "matches" << finalPass.finalMatches.size()
                     << "min" << std::max(4, settings.m_plateSolveMinMatches)
                     << "Az" << finalPass.pose.azimuthDegrees
                     << "El" << finalPass.pose.elevationDegrees
                     << "Roll" << finalPass.pose.rollDegrees;
        }
        return false;
    }

    const bool highConfidenceSparseAnchors =
        hasHighConfidenceSparseGuidedAnchors(settings, finalPass)
        || hasHighConfidenceGuidedTriangleSupport(settings, finalPass);
    if (hasWeakNarrowGuidedBrightSupport(settings, finalPass)
        && !highConfidenceSparseAnchors)
    {
        if (debugSparse)
        {
            qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: sparse final rejected by weak bright support"
                     << "matches" << finalPass.finalMatches.size()
                     << "brightDetections" << finalPass.matchedBrightDetections << "/" << finalPass.brightDetections
                     << "brightProjected" << finalPass.matchedBrightProjectedStars << "/" << finalPass.brightProjectedStars
                     << "seedBright" << finalPass.matchedSeedProjectedBrightStars << "/" << finalPass.seedProjectedBrightStars
                     << "sparseAnchors" << finalPass.sparseGuidedNamedAnchorMatches
                     << "anchorRms" << finalPass.sparseGuidedAnchorRmsErrorPixels
                     << "anchorBrightErr" << finalPass.sparseGuidedAnchorBrightnessRankError
                     << "Az" << finalPass.pose.azimuthDegrees
                     << "El" << finalPass.pose.elevationDegrees
                     << "Roll" << finalPass.pose.rollDegrees;
        }
        return false;
    }

    Evaluation evaluation = finalPass.pose;
    evaluation.matches = finalPass.finalMatches;
    evaluation.matchCount = finalPass.finalMatches.size();
    evaluation.rmsErrorPixels = finalPass.rmsErrorPixels;
    evaluation.brightnessRankError = finalPass.brightnessRankError;
    evaluation.meanCatalogMagnitude = finalPass.meanCatalogMagnitude;
    evaluation.valid = true;
    const bool accepted = isAcceptableSparseGuidedPairEvaluation(
        settings,
        catalogContext,
        starDetections,
        evaluation);
    if (debugSparse && !accepted)
    {
        qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: sparse final rejected by evaluation"
                 << "matches" << finalPass.finalMatches.size()
                 << "rms" << finalPass.rmsErrorPixels
                 << "Az" << finalPass.pose.azimuthDegrees
                 << "El" << finalPass.pose.elevationDegrees
                 << "Roll" << finalPass.pose.rollDegrees
                 << "primary" << finalPass.pose.anchorCatalogIndex << finalPass.pose.anchorDetectionIndex
                 << "secondary" << finalPass.pose.secondaryAnchorCatalogIndex << finalPass.pose.secondaryAnchorDetectionIndex;
    }
    return accepted;
}

CameraPlateSolver::SolverContext::Evaluation CameraPlateSolver::SolverContext::promoteSparseGuidedPairFromMatches(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QVector<CameraPipelineStarDetection>& starDetections, const Evaluation& candidate)
{
    Evaluation promoted;
    if (!candidate.valid
        || !plateSolveStartUsesDirection(settings)
        || plateSolveStartUsesRoll(settings)
        || isWidePlateSolveContext(settings)
        || (!isNarrowField(settings)))
    {
        return promoted;
    }

    QVector<Match> strongMatches;
    strongMatches.reserve(candidate.matches.size());
    const double maxAnchorDistance = std::min(
        static_cast<double>(settings.m_plateSolveFinalMatchRadius),
        24.0);
    for (const Match& match : candidate.matches)
    {
        if ((match.detectionIndex < 0)
            || (match.detectionIndex >= starDetections.size())
            || (match.catalogIndex < 0)
            || (match.catalogIndex >= catalogContext.catalogStars.size())
            || (match.distancePixels > maxAnchorDistance))
        {
            continue;
        }

        if (!isNamedSparseGuidedCatalogStar(catalogContext.catalogStars[match.catalogIndex])
            || !isStrongSparseGuidedDetection(starDetections[match.detectionIndex]))
        {
            continue;
        }

        strongMatches.append(match);
    }
    if (strongMatches.size() < 2) {
        return promoted;
    }

    std::sort(strongMatches.begin(), strongMatches.end(), [&catalogContext, &starDetections, this](const Match& lhs, const Match& rhs) {
        const auto scoreMatch = [&catalogContext, &starDetections, this](const Match& match) {
            const CatalogStar& catalogStar = catalogContext.catalogStars[match.catalogIndex];
            const CameraPipelineStarDetection& detection = starDetections[match.detectionIndex];
            return match.distancePixels
                + catalogStar.magnitude * 0.75
                - std::min(12.0, narrowGuidedAnchorShapeScore(detection) * 0.15)
                - std::min(8.0, std::log1p(cachedDetectionBrightnessMetric(starDetections, match.detectionIndex)) * 0.7);
        };
        const double lhsScore = scoreMatch(lhs);
        const double rhsScore = scoreMatch(rhs);
        if (!qFuzzyCompare(lhsScore + 1.0, rhsScore + 1.0)) {
            return lhsScore < rhsScore;
        }
        return lhs.distancePixels < rhs.distancePixels;
    });

    QVector<Match> anchors;
    anchors.reserve(2);
    QSet<int> usedDetections;
    QSet<int> usedCatalogStars;
    for (const Match& match : strongMatches)
    {
        if (usedDetections.contains(match.detectionIndex)
            || usedCatalogStars.contains(match.catalogIndex))
        {
            continue;
        }
        anchors.append(match);
        usedDetections.insert(match.detectionIndex);
        usedCatalogStars.insert(match.catalogIndex);
        if (anchors.size() >= 2) {
            break;
        }
    }
    if (anchors.size() < 2) {
        return promoted;
    }

    double sumSquaredError = 0.0;
    for (const Match& match : anchors) {
        sumSquaredError += match.distancePixels * match.distancePixels;
    }

    promoted = candidate;
    promoted.matches = anchors;
    promoted.matchCount = anchors.size();
    promoted.rmsErrorPixels = std::sqrt(sumSquaredError / static_cast<double>(anchors.size()));
    promoted.sparseGuidedPair = true;
    promoted.anchored = true;
    promoted.anchorDetectionIndex = anchors[0].detectionIndex;
    promoted.anchorCatalogIndex = anchors[0].catalogIndex;
    promoted.secondaryAnchorDetectionIndex = anchors[1].detectionIndex;
    promoted.secondaryAnchorCatalogIndex = anchors[1].catalogIndex;
    if (!isAcceptableSparseGuidedPairEvaluation(
            settings,
            catalogContext,
            starDetections,
            promoted))
    {
        promoted = Evaluation();
    }

    return promoted;
}

bool CameraPlateSolver::SolverContext::hasWeakNarrowGuidedBrightSupport(const CameraSettings& settings, const FinalMatchPassEvaluation& finalPass)
{
    if (!m_useDirectionSeedPreference
        || (!isNarrowField(settings))
        || !finalPass.projectorValid)
    {
        return false;
    }

    const bool useSeedProjectedBrightGate = usesSeedProjectedBrightGate(settings);
    const bool debugSparse = qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE");
    // Diagnostic only (SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE): logs which specific
    // sub-condition of this gate accepted/rejected a candidate, to make it possible to
    // trace why a particular pose (e.g. a bright-anchor-rescue candidate) was judged to
    // have weak/strong bright support.
    const auto logBrightSupportDecision = [&](bool weak, const char* reason) {
        if (!debugSparse) {
            return;
        }
        qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: hasWeakNarrowGuidedBrightSupport"
                 << (weak ? "REJECT" : "pass") << reason
                 << finalPassBrightDiagnosticSummary(finalPass)
                 << "Az" << finalPass.pose.azimuthDegrees
                 << "El" << finalPass.pose.elevationDegrees
                 << "Roll" << finalPass.pose.rollDegrees
                 << "FoV" << finalPass.pose.fovDegrees
                 << "useSeedProjectedBrightGate" << useSeedProjectedBrightGate
                 << "isLowMagnitude" << isLowMagnitudeNarrowGuidedSolve(settings);
    };

    // WS3 pass-2 (2026-06-20): the hasHighConfidenceGuidedTriangleSupport bypass was removed here --
    // ablation showed it casts no deciding vote (REAL 48, RAND2 148, FISHEYE-mode4 42, negatives clean)
    // even with strongDense still active, i.e. every pose it would bypass is already covered.
    // hasStrongDenseNarrowGuidedFinalPass is retained (load-bearing: disabling it drops galaxy-m31).
    // The function itself stays -- still used by isAcceptableSparseGuidedPairFinalPass.
    if (hasStrongDenseNarrowGuidedFinalPass(settings, finalPass) && !gateAblationDisabled("strongDense"))
    {
        logBrightSupportDecision(false, "strong-dense");
        return false;
    }
    if (hasNamedBrightAnchorCertifiedPose(settings, finalPass) && !gateAblationDisabled("namedAnchorCert"))
    {
        if (debugSparse)
        {
            qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: named-bright-anchor certificate bypasses weak-bright-support checks"
                     << "namedAnchors" << finalPass.namedBrightAnchorMatches
                     << "namedRms" << finalPass.namedBrightAnchorRmsErrorPixels
                     << "rms" << finalPass.rmsErrorPixels
                     << "matches" << finalPass.finalMatches.size()
                     << "seedOffsetDeg" << directionSeedAngularDistanceDegrees(finalPass.pose);
        }
        logBrightSupportDecision(false, "named-bright-anchor certified pose");
        return false;
    }
    if (hasSparseTightBrightCertifiedPose(settings, finalPass) && !gateAblationDisabled("sparseTightCert"))
    {
        logBrightSupportDecision(false, "sparse-tight bright certified pose");
        return false;
    }
    if (hasCompleteBrightAgreementPose(settings, finalPass) && !gateAblationDisabled("completeBrightAgreement"))
    {
        logBrightSupportDecision(false, "complete bright agreement certified pose");
        return false;
    }

    const bool denseFinalEvidenceOverridesSeedRadial =
        hasDenseFinalEvidenceOverridingSeedRadial(settings, finalPass);

    // Computed once and shared by both the low-magnitude branch below and the
    // general bright-support checks afterwards (previously duplicated verbatim in
    // both scopes with identical inputs).
    const bool poorNoRollSeedRadialSupport =
        hasPoorNoRollSeedRadialSupport(settings, finalPass, useSeedProjectedBrightGate);
    // The override exists because a bright star's detection can be *assigned* to a
    // nearby fainter catalog star in a dense catalog (undercounting matchedBright-
    // Detections) - but undercounting to ZERO while only faint projected stars match
    // is the signature of a wrong pose, not of assignment ambiguity (a wrong-FoV
    // carpet accept on pollux @0.4 deg matched 7/12 faint projected stars with 0/24
    // bright detections - the frame's genuinely bright stars, including Pollux
    // itself, matched nothing). Require at least one matched bright detection.
    const bool projectedBrightSupportCanOverrideDetectedBright =
        !useSeedProjectedBrightGate
        && !poorNoRollSeedRadialSupport
        && (finalPass.matchedBrightDetections >= 1)
        && (finalPass.matchedBrightProjectedStars >= 5)
        && (finalPass.projectedMagnitudeMatchFraction >= 0.30)
        && (finalPass.seedProjectedMagnitudeSupport < 80.0);

    if (isLowMagnitudeNarrowGuidedSolve(settings))
    {
        const bool projectedMagnitudeSupportGood =
            (finalPass.matchedProjectedMagnitudeSupport >= 12.0)
            && (finalPass.projectedMagnitudeMatchFraction >= 0.35);
        if ((finalPass.brightCatalogShapeChecks >= 1)
            && (finalPass.brightCatalogShapeMismatches > 0))
        {
            logBrightSupportDecision(true, "low-mag: brightCatalogShapeMismatch");
            return true;
        }
        if (poorNoRollSeedRadialSupport && !denseFinalEvidenceOverridesSeedRadial) {
            logBrightSupportDecision(true, "low-mag: poorNoRollSeedRadialSupport");
            return true;
        }
        if (useSeedProjectedBrightGate
            && (finalPass.seedProjectedBrightStars >= 2)
            && (finalPass.matchedSeedProjectedBrightStars < std::min(2, finalPass.seedProjectedBrightStars)))
        {
            logBrightSupportDecision(true, "low-mag: seedProjectedBright<2(gated)");
            return true;
        }
        if (!useSeedProjectedBrightGate
            && (finalPass.seedProjectedBrightStars >= 4)
            && (finalPass.matchedSeedProjectedBrightStars == 0)
            && (finalPass.seedProjectedMagnitudeSupport >= 80.0)
            && (finalPass.seedProjectedMagnitudeMatchFraction < 0.08))
        {
            logBrightSupportDecision(true, "low-mag: seedProjectedBright>=4 unmatched, weak magSupport");
            return true;
        }
        if ((finalPass.brightProjectedStars >= 4)
            && (finalPass.matchedBrightProjectedStars < 2))
        {
            logBrightSupportDecision(true, "low-mag: brightProjected>=4, matched<2");
            return true;
        }
        if ((finalPass.brightProjectedStars >= 10)
            && (finalPass.matchedBrightProjectedStars < 5)
            && !projectedMagnitudeSupportGood)
        {
            logBrightSupportDecision(true, "low-mag: brightProjected>=10, matched<5, weak magSupport");
            return true;
        }
        if ((finalPass.brightProjectedStars >= 6)
            && (finalPass.matchedBrightProjectedStars < 3))
        {
            logBrightSupportDecision(true, "low-mag: brightProjected>=6, matched<3");
            return true;
        }
        if ((finalPass.brightDetections >= 12)
            && (finalPass.matchedBrightDetections < 3)
            && !projectedBrightSupportCanOverrideDetectedBright)
        {
            logBrightSupportDecision(true, "low-mag: brightDetections>=12, matched<3");
            return true;
        }
        if (useSeedProjectedBrightGate
            && (finalPass.seedProjectedBrightStars >= 1)
            && (finalPass.matchedSeedProjectedBrightStars == 0))
        {
            logBrightSupportDecision(true, "low-mag: seedProjectedBright>=1 unmatched(gated)");
            return true;
        }
        if ((finalPass.brightDetections >= 6)
            && (finalPass.brightDetectionMagnitudeError > 2.25))
        {
            logBrightSupportDecision(true, "low-mag: brightDetectionMagnitudeError>2.25");
            return true;
        }
        if (!useSeedProjectedBrightGate
            && (finalPass.seedRadialMagnitudeSupport >= 60.0)
            && (finalPass.seedRadialMagnitudeMatchFraction < 0.04)
            && (finalPass.prioritySeedRadialChecks >= 12)
            && !denseFinalEvidenceOverridesSeedRadial
            && std::isfinite(finalPass.prioritySeedRadialErrorPixels)
            && (finalPass.prioritySeedRadialErrorPixels > std::max(
                    300.0,
                    static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 8.0)))
        {
            logBrightSupportDecision(true, "low-mag: seedRadialMagnitude weak + prioritySeedRadialError far");
            return true;
        }
        logBrightSupportDecision(false, "low-mag: passed all low-magnitude checks");
    }

    if (useSeedProjectedBrightGate
        && (finalPass.seedProjectedBrightStars >= 6)
        && (finalPass.matchedSeedProjectedBrightStars == 0))
    {
        const int minimumBrightDetectionMatches = std::min(
            8,
            std::max(3, finalPass.brightDetections / 3));
        if (finalPass.matchedBrightDetections < minimumBrightDetectionMatches) {
            logBrightSupportDecision(true, "seedProjectedBright>=6 unmatched, brightDetections below floor(/3,min3)");
            return true;
        }
    }
    if (useSeedProjectedBrightGate
        && (finalPass.seedProjectedBrightStars >= 10)
        && (finalPass.matchedSeedProjectedBrightStars <= 1))
    {
        const int minimumBrightDetectionMatches = std::min(
            8,
            std::max(4, finalPass.brightDetections / 3));
        if (finalPass.matchedBrightDetections < minimumBrightDetectionMatches) {
            logBrightSupportDecision(true, "seedProjectedBright>=10 <=1 matched, brightDetections below floor(/3,min4)");
            return true;
        }
    }

    const bool weakBrightDetections =
        (finalPass.brightDetections >= 6)
        && (finalPass.matchedBrightDetections < 2)
        && !projectedBrightSupportCanOverrideDetectedBright;
    const bool weakSeedRadial = poorNoRollSeedRadialSupport && !denseFinalEvidenceOverridesSeedRadial;
    // (WS3 2026-06-21) Removed the strongBrightDetectionSupportWideFisheye waiver that used to gate
    // weakBrightProjected: it was provably dead here. This function early-returns for !isNarrowField
    // (fov > 5 deg), and reaching this point also requires a direction-seeded (non-blind) solve, so
    // isWidePlateSolveContext -- which needs fov >= 30 deg or blind mode -- is always false at this
    // point, making the waiver always false. Firing analysis confirmed 0 fires across REAL+RAND2 and
    // FISH4 (wide fisheye) never enters this narrow-only function. The live wide-fisheye
    // bright-projected waiver is in hasAcceptableGuidedFinalBrightnessConsistency.
    const bool weakBrightProjected =
        (finalPass.brightProjectedStars >= 5)
        && (finalPass.matchedBrightProjectedStars < 2);
    const bool weakBrightMagnitude =
        (finalPass.brightDetections >= 6) && (finalPass.brightDetectionMagnitudeError > 2.35);
    const bool weak = weakBrightDetections || weakSeedRadial || weakBrightProjected || weakBrightMagnitude;
    if (debugSparse && weak)
    {
        qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: hasWeakNarrowGuidedBrightSupport REJECT final"
                 << "weakBrightDetections" << weakBrightDetections
                 << "weakSeedRadial" << weakSeedRadial
                 << "weakBrightProjected" << weakBrightProjected
                 << "weakBrightMagnitude" << weakBrightMagnitude
                 << finalPassBrightDiagnosticSummary(finalPass)
                 << "Az" << finalPass.pose.azimuthDegrees
                 << "El" << finalPass.pose.elevationDegrees
                 << "Roll" << finalPass.pose.rollDegrees
                 << "FoV" << finalPass.pose.fovDegrees;
    }
    else
    {
        logBrightSupportDecision(false, "passed final compound check");
    }
    return weak;
}

bool CameraPlateSolver::SolverContext::isAcceptableSparseGuidedRankingFinalPass(const CameraSettings& settings, const FinalMatchPassEvaluation& finalPass)
{
    if (!finalPass.projectorValid
        || !finalPass.pose.sparseGuidedPair
        || !plateSolveStartUsesDirection(settings)
        || isWidePlateSolveContext(settings)
        || (!isNarrowField(settings))
        || (finalPass.finalMatches.size() < std::max(4, settings.m_plateSolveMinMatches))
        || (finalPass.matchedBrightProjectedStars < 2))
    {
        return false;
    }

    if (hasWeakNarrowGuidedBrightSupport(settings, finalPass)) {
        return false;
    }

    const double maxRmsError = sparseGuidedMaxRms(settings, static_cast<int>(finalPass.finalMatches.size()));

    return std::isfinite(finalPass.rmsErrorPixels)
        && (finalPass.rmsErrorPixels <= maxRmsError);
}

bool CameraPlateSolver::SolverContext::hasStrongDenseNarrowGuidedFinalPass(const CameraSettings& settings, const FinalMatchPassEvaluation& finalPass)
{
    if (!m_useDirectionSeedPreference
        || (!isNarrowField(settings))
        || !finalPass.projectorValid
        || (finalPass.finalMatches.size() < static_cast<qsizetype>(std::max(settings.m_plateSolveMinMatches + 18, 24)))
        || !std::isfinite(finalPass.rmsErrorPixels)
        || (finalPass.rmsErrorPixels > maxDirectionSeedRmsError(settings, finalPass.finalMatches.size())))
    {
        return false;
    }

    const bool veryDenseMatchSet =
        finalPass.finalMatches.size() >= static_cast<qsizetype>(std::max(settings.m_plateSolveMinMatches + 76, 96));
    const int projectedBrightThreshold = std::min(
        finalPass.brightProjectedStars,
        std::max(veryDenseMatchSet ? 3 : 5,
            static_cast<int>(std::ceil(finalPass.brightProjectedStars * (veryDenseMatchSet ? 0.45 : 0.70)))));
    const int detectedBrightThreshold = std::min(
        finalPass.brightDetections,
        std::max(veryDenseMatchSet ? 3 : 6,
            static_cast<int>(std::ceil(finalPass.brightDetections * (veryDenseMatchSet ? 0.20 : 0.33)))));
    const double projectedMagnitudeThreshold = veryDenseMatchSet ? 0.18 : 0.70;
    const double seedProjectedMagnitudeThreshold = veryDenseMatchSet ? 0.18 : 0.60;
    return (finalPass.brightProjectedStars >= 6)
        && (finalPass.matchedBrightProjectedStars >= projectedBrightThreshold)
        && (finalPass.matchedBrightDetections >= detectedBrightThreshold)
        && (finalPass.projectedMagnitudeMatchFraction >= projectedMagnitudeThreshold)
        && ((finalPass.seedProjectedMagnitudeSupport <= 0.0)
            || (finalPass.seedProjectedMagnitudeMatchFraction >= seedProjectedMagnitudeThreshold)
            || veryDenseMatchSet)
        && ((finalPass.brightDetectionMagnitudeError <= 1.60)
            || (finalPass.matchedBrightDetections >= std::min(finalPass.brightDetections, 12)));
}

double CameraPlateSolver::SolverContext::evaluationRmsQuality(const Evaluation& evaluation, double normalizationRadius)
{
    const double safeRadius = std::max(1.0, normalizationRadius);
    const double normalizedRms = evaluation.rmsErrorPixels / safeRadius;
    const double clampedRms = std::min(1.0, std::max(0.0, normalizedRms));
    return 1.0 - 0.5 * clampedRms * clampedRms;
}

double CameraPlateSolver::SolverContext::brightnessAffinity(const Evaluation& evaluation)
{
    if (!std::isfinite(evaluation.brightnessRankError) || (evaluation.matches.size() < 3)) {
        return 1.0;
    }

    const double clampedError = std::min(1.0, std::max(0.0, evaluation.brightnessRankError));
    return 1.0 / (1.0 + 3.0 * clampedError * clampedError);
}

double CameraPlateSolver::SolverContext::catalogMagnitudeAffinity(const Evaluation& evaluation)
{
    if (!m_useWideCatalogMagnitudePreference
        || !std::isfinite(evaluation.meanCatalogMagnitude)
        || (evaluation.matches.size() < 3))
    {
        return 1.0;
    }

    const double faintExcess = std::max(0.0, evaluation.meanCatalogMagnitude - 2.0);
    return 1.0 / (1.0 + 0.65 * faintExcess * faintExcess);
}

double CameraPlateSolver::SolverContext::wideEvaluationMatchWeight(const Evaluation& evaluation)
{
    if (!m_useWideCatalogMagnitudePreference) {
        return static_cast<double>(evaluation.matchCount);
    }

    const int usefulMatchCap = std::max(m_directionSeedMinMatchCount + 4, 8);
    const int cappedMatches = std::min(evaluation.matchCount, usefulMatchCap);
    const int extraMatches = std::max(0, evaluation.matchCount - usefulMatchCap);
    return static_cast<double>(cappedMatches) + 0.15 * std::log1p(static_cast<double>(extraMatches));
}

bool CameraPlateSolver::SolverContext::hasAcceptableBrightnessConsistency(const Evaluation& evaluation)
{
    return !std::isfinite(evaluation.brightnessRankError)
        || (evaluation.matches.size() < 4)
        || (evaluation.brightnessRankError <= 0.45);
}

bool CameraPlateSolver::SolverContext::hasAcceptableGuidedFinalBrightnessConsistency(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation)
{
    if (!std::isfinite(evaluation.brightnessRankError) || (evaluation.finalMatches.size() < 4)) {
        return true;
    }

    if (isNarrowField(settings)) {
        if ((evaluation.brightCatalogShapeChecks >= 2)
            && (evaluation.brightCatalogShapeMismatches > 0))
        {
            return false;
        }
        // A sparse-tight certified pose is geometrically verified (every match tight,
        // named anchor pinned, near-seed); the seed-radial/brightness-rank heuristics
        // below assume more matches than a sparse bright-only solve can produce.
        if (hasSparseTightBrightCertifiedPose(settings, evaluation) && !gateAblationDisabled("sparseTightCert")) {
            return true;
        }
        // Likewise when every bright star in the frame is accounted for - there is no
        // brightness inconsistency left to find
        if (hasCompleteBrightAgreementPose(settings, evaluation) && !gateAblationDisabled("completeBrightAgreement")) {
            return true;
        }

        const bool lowMagnitudeNarrowGuided = isLowMagnitudeNarrowGuidedSolve(settings);
        const bool useSeedProjectedBrightGate = usesSeedProjectedBrightGate(settings);
        const bool poorNoRollSeedRadialSupport =
            hasPoorNoRollSeedRadialSupport(settings, evaluation, useSeedProjectedBrightGate)
            && !hasNamedBrightAnchorCertifiedPose(settings, evaluation);
        const bool projectedBrightSupportCanOverrideDetectedBright =
            !useSeedProjectedBrightGate
            && !poorNoRollSeedRadialSupport
            && (evaluation.matchedBrightDetections >= 1)
            && (evaluation.matchedBrightProjectedStars >= 5)
            && (evaluation.projectedMagnitudeMatchFraction >= 0.30)
            && (evaluation.seedProjectedMagnitudeSupport < 80.0);
        if (lowMagnitudeNarrowGuided
            && poorNoRollSeedRadialSupport
            && !hasDenseFinalEvidenceOverridingSeedRadial(settings, evaluation)) {
            return false;
        }
        if (useSeedProjectedBrightGate
            && lowMagnitudeNarrowGuided
            && (evaluation.seedProjectedBrightStars > 0)
            && (evaluation.matchedSeedProjectedBrightStars == 0))
        {
            const bool strongProjectedBrightSupport =
                (evaluation.brightProjectedStars >= 4)
                && (evaluation.brightProjectedMatchFraction >= 0.85)
                && ((evaluation.brightDetections < 6)
                    || (evaluation.brightDetectionMatchFraction >= 0.65))
                && ((evaluation.brightDetectionMagnitudeError <= 1.50)
                    || (evaluation.matchedBrightProjectedStars >= 6));
            const bool strongDetectedBrightSupport =
                (evaluation.brightDetections >= 8)
                && (evaluation.brightDetectionMatchFraction >= 0.65)
                && (evaluation.brightDetectionMagnitudeError <= 0.45);
            if (!strongProjectedBrightSupport && !strongDetectedBrightSupport) {
                return false;
            }
        }

        if ((evaluation.finalMatches.size() >= 20)
            && (evaluation.brightnessRankError <= 0.35))
        {
            return true;
        }

        if (lowMagnitudeNarrowGuided
            && (evaluation.finalMatches.size() >= 16)
            && (evaluation.rmsErrorPixels > std::min(settings.m_plateSolveFinalMatchRadius * 0.62, 14.5))
            && (evaluation.brightProjectedStars >= 3)
            && (evaluation.matchedBrightProjectedStars >= 2)
            && (evaluation.brightDetectionMagnitudeError <= 1.45)
            && (evaluation.brightCatalogShapeMismatches == 0)
            && (!std::isfinite(evaluation.brightnessRankError) || (evaluation.brightnessRankError <= 0.70)))
        {
            return true;
        }
        if ((evaluation.brightProjectedStars >= 2)
            && (evaluation.matchedBrightProjectedStars == 0))
        {
            return false;
        }
        if ((evaluation.brightProjectedStars >= 5)
            && (evaluation.matchedBrightProjectedStars < 2))
        {
            return false;
        }
        if (lowMagnitudeNarrowGuided
            && (evaluation.brightDetections >= 12)
            && (evaluation.matchedBrightDetections < 3)
            && !projectedBrightSupportCanOverrideDetectedBright)
        {
            return false;
        }

        // For narrow-field (telescope) solves the FOV is pinned to the user's value,
        // which strongly constrains the geometry.  Brightness rank ordering is also less
        // reliable when the matched set spans mag 2-13 (saturated bright star + very faint
        // stars).  When at least two bright projected stars are retained, give that
        // geometric bright-star support a little more weight than the global rank order.
        const double threshold =
            ((evaluation.matchedBrightProjectedStars >= 2)
                && (evaluation.brightDetectionMagnitudeError <= 1.60))
            ? 0.70
            : 0.65;
        return evaluation.brightnessRankError <= threshold;
    }

    const double threshold = (settings.m_fov <= 30.0) ? 0.50
        : 0.60;
    if (evaluation.brightnessRankError > threshold) {
        return false;
    }

    if (isWidePlateSolveContext(settings)
        && (evaluation.brightDetections >= 5)
        && (evaluation.brightDetectionMagnitudeError > 0.85))
    {
        // A dense, low-RMS geometric fit corroborates the pose independently of
        // photometric rank consistency, which is noisier in wide fisheye (rendered/real
        // brightness vs catalogue magnitude scatter, plus the brightest catalogue stars
        // often falling beyond the image circle). Don't reject a clearly-correct guided
        // solve (many matches at sub-radius RMS) on magnitude error alone - e.g. the
        // synth-fisheye-027/047 guided solves sit at magErr ~0.9-1.1 with rms 1-3 px.
        const bool strongGeometricFit =
            (evaluation.finalMatches.size() >= 20)
            && std::isfinite(evaluation.rmsErrorPixels)
            && (evaluation.rmsErrorPixels <= std::min(settings.m_plateSolveFinalMatchRadius * 0.40, 8.0))
            && (evaluation.brightDetectionMagnitudeError <= 1.50);
        if (!strongGeometricFit) {
            return false;
        }
    }

    return true;
}

bool CameraPlateSolver::SolverContext::needsWideBrightAnchorSupport(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections)
{
    return isWidePlateSolveContext(settings)
        && (starDetections.size() >= 32);
}

bool CameraPlateSolver::SolverContext::hasAcceptableWideBrightAnchorSupport(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections, const FinalMatchPassEvaluation& evaluation)
{
    if (!needsWideBrightAnchorSupport(settings, starDetections)) {
        return true;
    }
    if (evaluation.brightDetections < 5) {
        return false;
    }

    const bool brightDetectionsAgree =
        (evaluation.matchedBrightDetections >= 4)
        && (evaluation.brightDetectionMagnitudeError <= 1.50);
    const bool brightDetectionsStronglyAgree =
        (evaluation.finalMatches.size() >= std::max(settings.m_plateSolveMinMatches + 4, 8))
        && (evaluation.brightDetections >= 6)
        && (evaluation.matchedBrightDetections >= 6)
        && (evaluation.brightDetectionMatchFraction >= 0.75)
        && (evaluation.brightDetectionMagnitudeError <= 0.25);
    const bool brightProjectedStarsAgree =
        (evaluation.brightProjectedStars < 5)
        || (evaluation.matchedBrightProjectedStars >= 2)
        || (evaluation.brightProjectedMatchFraction >= 0.25);

    if (settings.m_plateSolveStartMode == CameraSettings::PlateSolveStartBlind) {
        return brightDetectionsStronglyAgree
            || (brightDetectionsAgree && brightProjectedStarsAgree);
    }

    // Wide direction-less (FoV / FoV+elevation start, no azimuth prior) brightness-consistency
    // floor. The bright-detection / bright-projected agreement above is evadable by structured-blob
    // garbage: a wrong wide pose over neg-blobs (uniform detection flux) can land 5 of its 8 bright
    // blobs on bright catalog stars and even match 2 bright projected stars, satisfying the tests
    // below, while its detection<->catalog brightness ordering stays scrambled (brightnessRankError
    // ~0.29-0.32). Genuine wide-fisheye solves separate cleanly: every one that carries a high rank
    // error matches >= 7 bright detections (dense all-sky fisheye, e.g. stars-wide-3 at rankError
    // 0.30 / 8 bright matched), and the only genuine direction-less solves that lean on <= 5 bright
    // matches keep a consistent ordering (<= 0.21). So require either >= 6 matched bright detections
    // or a consistent brightness ordering; neg-blobs has neither.
    //
    // The floor is scoped to the direction-less start modes. When the user supplies an azimuth/
    // elevation prior (FovAzEl and up) the pose is already pinned, so the search cannot wander onto
    // a brightness-scrambled garbage basin; there the floor only risks vetoing a genuinely faint
    // guided field whose correct pose legitimately matches few bright stars with a high rank error
    // (e.g. synth-fisheye-044 in mode 4: 4 bright matched, rankError 0.35).
    const bool brightnessOrderingSupported =
        plateSolveStartUsesDirection(settings)
        || (evaluation.matchedBrightDetections >= 6)
        || !std::isfinite(evaluation.brightnessRankError)
        || (evaluation.brightnessRankError <= 0.25);

    return brightnessOrderingSupported
        && brightDetectionsAgree
        && (brightProjectedStarsAgree || (evaluation.matchedBrightDetections >= 4));
}

bool CameraPlateSolver::SolverContext::isStrongEarlyGuidedFinalPass(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections, const FinalMatchPassEvaluation& evaluation, int requiredMatches, double finalMatchRadius)
{
    if (!m_useDirectionSeedPreference || !evaluation.projectorValid) {
        return false;
    }

    // A roll prior can make an early final pass look convincing while still
    // sitting in the wrong local basin. Let narrow roll-constrained solves run
    // the multi-hypothesis comparison before accepting a solution.
    if ((isNarrowField(settings)) && plateSolveStartUsesRoll(settings)) {
        return false;
    }
    if (!isAcceptableNarrowGuidedFov(settings, evaluation.pose.fovDegrees)) {
        return false;
    }

    const int minimumEarlyMatches = (isNarrowField(settings))
        ? std::max(requiredMatches + 12, 16)
        : std::max(requiredMatches + 6, 10);
    if (evaluation.finalMatches.size() < minimumEarlyMatches) {
        return false;
    }

    const bool denseNarrowHighSupport =
        (isNarrowField(settings))
        && (evaluation.finalMatches.size() >= std::max(requiredMatches + 48, 64))
        && (evaluation.rmsErrorPixels <= std::min(maxDirectionSeedRmsError(settings, evaluation.finalMatches.size()) * 0.95, finalMatchRadius * 0.70))
        && (evaluation.maxErrorPixels <= std::min(finalMatchRadius * 1.05, 30.0));
    if (denseNarrowHighSupport
        && isAcceptableDirectionSeedSolve(
            settings,
            starDetections,
            evaluation.finalMatches,
            evaluation.rmsErrorPixels,
            evaluation.maxErrorPixels)
        && hasGeometricallyConsistentMatches(
            starDetections,
            evaluation.projectedStars,
            evaluation.finalMatches,
            finalMatchRadius))
    {
        return true;
    }

    if (!hasAcceptableWideBrightAnchorSupport(settings, starDetections, evaluation)
        || hasWeakNarrowGuidedBrightSupport(settings, evaluation)
        || !hasAcceptableGuidedFinalBrightnessConsistency(settings, evaluation))
    {
        return false;
    }

    if (!isAcceptableDirectionSeedSolve(
            settings,
            starDetections,
            evaluation.finalMatches,
            evaluation.rmsErrorPixels,
            evaluation.maxErrorPixels))
    {
        return false;
    }

    return hasGeometricallyConsistentMatches(
        starDetections,
        evaluation.projectedStars,
        evaluation.finalMatches,
        finalMatchRadius);
}

double CameraPlateSolver::SolverContext::directionSeedAffinity(const Evaluation& evaluation)
{
    if (!m_useDirectionSeedPreference) {
        return 1.0;
    }

    const double safeAzElScale = std::max(1.0, m_directionSeedAzElScaleDegrees);
    const double safeRollScale = std::max(1.0, m_directionSeedRollScaleDegrees);
    const double safeFovScale = std::max(1.0, m_directionSeedFovScaleDegrees);
    const double normalizedAzimuthDelta = angularDistanceDegrees(
        evaluation.azimuthDegrees,
        m_directionSeedReferenceAzimuthDegrees) / safeAzElScale;
    const double normalizedElevationDelta = std::fabs(
        evaluation.elevationDegrees - m_directionSeedReferenceElevationDegrees) / safeAzElScale;
    const double normalizedRollDelta = m_directionSeedHasRollPreference
        ? angularDistanceDegrees(evaluation.rollDegrees, m_directionSeedReferenceRollDegrees) / safeRollScale
        : 0.0;
    const double normalizedFovDelta = std::fabs(
        evaluation.fovDegrees - m_directionSeedReferenceFovDegrees) / safeFovScale;

    return 1.0 / (1.0
        + 0.35 * normalizedAzimuthDelta * normalizedAzimuthDelta
        + 0.35 * normalizedElevationDelta * normalizedElevationDelta
        + 0.18 * normalizedRollDelta * normalizedRollDelta
        + 0.75 * normalizedFovDelta * normalizedFovDelta);
}

double CameraPlateSolver::SolverContext::fovSeedAffinity(const Evaluation& evaluation)
{
    if (!m_useFovSeedPreference) {
        return 1.0;
    }

    const double safeScale = std::max(1.0, m_fovSeedScaleDegrees);
    const double normalizedFovDelta = std::fabs(
        evaluation.fovDegrees - m_fovSeedReferenceDegrees) / safeScale;
    return 1.0 / (1.0 + 1.25 * normalizedFovDelta * normalizedFovDelta);
}

double CameraPlateSolver::SolverContext::allSkyZenithAffinity(const Evaluation& evaluation)
{
    if (!m_useAllSkyZenithPreference) {
        return 1.0;
    }

    const double safeScale = std::max(5.0, m_allSkyZenithScaleDegrees);
    const double normalizedElevationDelta =
        std::fabs(evaluation.elevationDegrees - m_allSkyZenithReferenceElevationDegrees) / safeScale;
    return 1.0 / (1.0 + 1.5 * normalizedElevationDelta * normalizedElevationDelta);
}

double CameraPlateSolver::SolverContext::guidedDirectionEvaluationScore(const Evaluation& evaluation, double normalizationRadius)
{
    if (normalizationRadius < 0.0) {
        normalizationRadius = m_weakModeNormalizationPixels;
    }
    if (!evaluation.valid) {
        return -std::numeric_limits<double>::infinity();
    }

    return wideEvaluationMatchWeight(evaluation)
        * evaluationRmsQuality(evaluation, normalizationRadius)
        * brightnessAffinity(evaluation)
        * catalogMagnitudeAffinity(evaluation)
        * directionSeedAffinity(evaluation)
        * fovSeedAffinity(evaluation)
        * allSkyZenithAffinity(evaluation);
}

double CameraPlateSolver::SolverContext::weakModeEvaluationScore(const Evaluation& evaluation, double normalizationRadius)
{
    if (normalizationRadius < 0.0) {
        normalizationRadius = m_weakModeNormalizationPixels;
    }
    if (!evaluation.valid) {
        return -std::numeric_limits<double>::infinity();
    }

    // Non-linear penalty proportional to the acceptance radius: a small RMS leaves the score
    // essentially equal to matchCount, but as RMS approaches the radius the per-match value
    // collapses to ~0.5 so a single false coincidence cannot outweigh a tighter cluster.
    double score = wideEvaluationMatchWeight(evaluation)
        * evaluationRmsQuality(evaluation, normalizationRadius)
        * brightnessAffinity(evaluation)
        * catalogMagnitudeAffinity(evaluation)
        * allSkyZenithAffinity(evaluation);

    if (m_useElevationSeedPreference)
    {
        const double safeElevationScale = std::max(1.0, m_elevationSeedScaleDegrees);
        const double safeFovScale = std::max(1.0, m_elevationSeedFovScaleDegrees);
        const double normalizedElevationDelta = std::fabs(
            evaluation.elevationDegrees - m_elevationSeedReferenceDegrees) / safeElevationScale;
        const double normalizedFovDelta = std::fabs(
            evaluation.fovDegrees - m_elevationSeedReferenceFovDegrees) / safeFovScale;
        const double seedAffinity = 1.0 / (1.0
            + 0.5 * normalizedElevationDelta * normalizedElevationDelta
            + 1.5 * normalizedFovDelta * normalizedFovDelta);
        score *= seedAffinity;
    }

    return score;
}

bool CameraPlateSolver::SolverContext::isBetterWeakModeEvaluation(const Evaluation& candidate, const Evaluation& best)
{
    if (!candidate.valid) {
        return false;
    }
    if (!best.valid) {
        return true;
    }

    const double candidateScore = weakModeEvaluationScore(candidate);
    const double bestScore = weakModeEvaluationScore(best);
    if (std::fabs(candidateScore - bestScore) > 1e-6) {
        return candidateScore > bestScore;
    }

    return isBetterByGeometricTieBreak(candidate, best);
}

bool CameraPlateSolver::SolverContext::isBetterGuidedDirectionEvaluation(const Evaluation& candidate, const Evaluation& best)
{
    if (!candidate.valid) {
        return false;
    }
    if (!best.valid) {
        return true;
    }

    const bool candidateMeetsMinMatches = candidate.matchCount >= m_directionSeedMinMatchCount;
    const bool bestMeetsMinMatches = best.matchCount >= m_directionSeedMinMatchCount;
    if (candidateMeetsMinMatches != bestMeetsMinMatches) {
        return candidateMeetsMinMatches;
    }

    const double candidateScore = guidedDirectionEvaluationScore(candidate);
    const double bestScore = guidedDirectionEvaluationScore(best);
    if (std::fabs(candidateScore - bestScore) > 0.05) {
        return candidateScore > bestScore;
    }

    if (candidate.matchCount != best.matchCount) {
        return candidate.matchCount > best.matchCount;
    }
    if (!qFuzzyCompare(candidate.rmsErrorPixels + 1.0, best.rmsErrorPixels + 1.0)) {
        return candidate.rmsErrorPixels < best.rmsErrorPixels;
    }
    if (!qFuzzyCompare(candidate.brightnessRankError + 1.0, best.brightnessRankError + 1.0)) {
        return candidate.brightnessRankError < best.brightnessRankError;
    }

    return isBetterEvaluation(candidate, best);
}

bool CameraPlateSolver::SolverContext::isBetterWeakModeRefinedEvaluation(const Evaluation& candidate, const Evaluation& best)
{
    if (!candidate.valid) {
        return false;
    }
    if (!best.valid) {
        return true;
    }

    const int matchDelta = candidate.matchCount - best.matchCount;
    if (std::abs(matchDelta) <= 1)
    {
        const double candidateMedian = medianDistancePixels(candidate.matches);
        const double bestMedian = medianDistancePixels(best.matches);
        if (!qFuzzyCompare(candidateMedian + 1.0, bestMedian + 1.0)) {
            return candidateMedian < bestMedian;
        }
        if (!qFuzzyCompare(candidate.rmsErrorPixels + 1.0, best.rmsErrorPixels + 1.0)) {
            return candidate.rmsErrorPixels < best.rmsErrorPixels;
        }
    }

    return isBetterWeakModeEvaluation(candidate, best);
}

const CameraPlateSolver::SolverContext::FinalPassSeedCache&
CameraPlateSolver::SolverContext::finalPassSeedCache(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QVector<CameraPipelineStarDetection>& starDetections)
{
    FinalPassSeedCache& cache = m_finalPassSeedCache;
    const bool useNarrowGuidedBrightPrior = m_useDirectionSeedPreference && isNarrowField(settings);
    const bool useSeedRadialPrior = useNarrowGuidedBrightPrior && !m_directionSeedHasRollPreference;
    const int detectionCount = static_cast<int>(starDetections.size());

    // Reuse the cache only when every dependency of the stored quantities is unchanged. The guard
    // errs toward rebuilding: any mismatch recomputes. Catalog content is tracked by the generation
    // stamp (bumped on every visibleStars (re)populate); the seed reference and lens are exact.
    if (cache.valid
        && (cache.catalogGeneration == catalogContext.visibleStarsGeneration)
        && (cache.detectionCount == detectionCount)
        && (cache.useNarrowGuidedBrightPrior == useNarrowGuidedBrightPrior)
        && (cache.useSeedRadialPrior == useSeedRadialPrior)
        && (cache.refAz == m_directionSeedReferenceAzimuthDegrees)
        && (cache.refEl == m_directionSeedReferenceElevationDegrees)
        && (cache.refRoll == m_directionSeedReferenceRollDegrees)
        && (cache.refFov == m_directionSeedReferenceFovDegrees)
        && (cache.lensCx == settings.m_lensCenterOffsetX)
        && (cache.lensCy == settings.m_lensCenterOffsetY)
        && (cache.lensK1 == settings.m_lensDistortionK1))
    {
        return cache;
    }

    cache = FinalPassSeedCache{};
    cache.catalogGeneration = catalogContext.visibleStarsGeneration;
    cache.detectionCount = detectionCount;
    cache.useNarrowGuidedBrightPrior = useNarrowGuidedBrightPrior;
    cache.useSeedRadialPrior = useSeedRadialPrior;
    cache.refAz = m_directionSeedReferenceAzimuthDegrees;
    cache.refEl = m_directionSeedReferenceElevationDegrees;
    cache.refRoll = m_directionSeedReferenceRollDegrees;
    cache.refFov = m_directionSeedReferenceFovDegrees;
    cache.lensCx = settings.m_lensCenterOffsetX;
    cache.lensCy = settings.m_lensCenterOffsetY;
    cache.lensK1 = settings.m_lensDistortionK1;

    // Mirrors the former inline construction in evaluateFinalMatchPass exactly.
    cache.seedRadialProjector = useSeedRadialPrior
        ? createProjector(
            settings, imageSize,
            m_directionSeedReferenceAzimuthDegrees, m_directionSeedReferenceElevationDegrees,
            m_directionSeedReferenceRollDegrees, m_directionSeedReferenceFovDegrees,
            settings.m_lensCenterOffsetX, settings.m_lensCenterOffsetY, settings.m_lensDistortionK1)
        : SkyProjector();
    cache.seedRadialCenter = projectorPrincipalPoint(cache.seedRadialProjector);

    if (cache.seedRadialProjector.valid)
    {
        const int visibleCount = static_cast<int>(catalogContext.visibleStars.size());
        cache.seedPoints.resize(visibleCount);
        cache.seedPointValid.resize(visibleCount);
        for (int i = 0; i < visibleCount; ++i)
        {
            QPointF seedPoint;
            const bool ok = projectVector(cache.seedRadialProjector, catalogContext.visibleStars[i].vector, seedPoint);
            cache.seedPoints[i] = seedPoint;
            cache.seedPointValid[i] = ok ? quint8(1) : quint8(0);
        }
        cache.sortedDetectionRadii.reserve(detectionCount);
        for (const CameraPipelineStarDetection& detection : starDetections) {
            cache.sortedDetectionRadii.append(pointDistancePixels(detection.m_center, cache.seedRadialCenter));
        }
        std::sort(cache.sortedDetectionRadii.begin(), cache.sortedDetectionRadii.end());
    }

    if (useNarrowGuidedBrightPrior)
    {
        cache.brightDetectionIndices.reserve(detectionCount);
        for (int detectionIndex = 0; detectionIndex < detectionCount; ++detectionIndex) {
            cache.brightDetectionIndices.append(detectionIndex);
        }
        cache.brightDetectionIndices.erase(
            std::remove_if(
                cache.brightDetectionIndices.begin(),
                cache.brightDetectionIndices.end(),
                [&starDetections](int detectionIndex) {
                    return (detectionIndex < 0)
                        || (detectionIndex >= starDetections.size())
                        || !isDetectionUsableForBrightPrior(starDetections[detectionIndex]);
                }),
            cache.brightDetectionIndices.end());
        std::sort(cache.brightDetectionIndices.begin(), cache.brightDetectionIndices.end(), [this, &starDetections](int lhs, int rhs) {
            const double lhsBrightness = cachedDetectionBrightnessMetric(starDetections, lhs);
            const double rhsBrightness = cachedDetectionBrightnessMetric(starDetections, rhs);
            const double lhsReliability = cachedDetectionReliabilityMetric(starDetections, lhs);
            const double rhsReliability = cachedDetectionReliabilityMetric(starDetections, rhs);
            const double lhsImportance = lhsBrightness * std::sqrt(std::max(0.0, lhsReliability));
            const double rhsImportance = rhsBrightness * std::sqrt(std::max(0.0, rhsReliability));
            if (!qFuzzyCompare(lhsImportance + 1.0, rhsImportance + 1.0)) {
                return lhsImportance > rhsImportance;
            }
            return lhsBrightness > rhsBrightness;
        });
    }

    cache.valid = true;
    return cache;
}

CameraPlateSolver::SolverContext::FinalMatchPassEvaluation CameraPlateSolver::SolverContext::evaluateFinalMatchPass(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const Evaluation& candidate, double finalMatchRadius, bool restrictSupplementalMatchesToDetectionIndices)
{
    FinalMatchPassEvaluation finalPass;
    finalPass.pose = candidate;
    const FinalPassSeedCache& seedCache = finalPassSeedCache(settings, catalogContext, imageSize, starDetections);

    const SkyProjector projector = createProjector(
        settings,
        imageSize,
        candidate.azimuthDegrees,
        candidate.elevationDegrees,
        candidate.rollDegrees,
        candidate.fovDegrees,
        candidate.centerOffsetXPixels,
        candidate.centerOffsetYPixels,
        candidate.distortionK1);
    if (!projector.valid) {
        return finalPass;
    }

    finalPass.projectorValid = true;
    finalPass.projectedStars = buildProjectedCatalog(
        catalogContext,
        projector,
        finalMatchRadius);

    QVector<Match> allMatches;
    QVector<Match> forcedAnchorMatches;
    bool rejectAnchoredCandidate = false;
    if (candidate.anchored)
    {
        QSet<int> matchedDetections;
        QSet<int> matchedCatalogStars;
        const auto appendForcedAnchor = [&](int detectionIndex, int catalogIndex) {
            if ((detectionIndex < 0)
                || (detectionIndex >= starDetections.size())
                || (catalogIndex < 0)
                || (catalogIndex >= catalogContext.catalogStars.size())
                || matchedDetections.contains(detectionIndex)
                || matchedCatalogStars.contains(catalogIndex))
            {
                return false;
            }

            int projectedIndex = -1;
            for (int i = 0; i < finalPass.projectedStars.size(); ++i)
            {
                if (finalPass.projectedStars[i].catalogIndex == catalogIndex)
                {
                    projectedIndex = i;
                    break;
                }
            }
            if (projectedIndex < 0) {
                return false;
            }

            const double anchorDistance = pointDistancePixels(
                starDetections[detectionIndex].m_center,
                finalPass.projectedStars[projectedIndex].point);
            const bool anchorHasImplausibleBrightShape =
                isImplausiblyCompactBrightCatalogDetection(
                    starDetections[detectionIndex],
                    catalogContext.catalogStars[catalogIndex].magnitude,
                    m_useDirectionSeedPreference && (isNarrowField(settings)));
            if ((anchorDistance > finalMatchRadius) || anchorHasImplausibleBrightShape) {
                return false;
            }

            allMatches.append({detectionIndex, catalogIndex, anchorDistance});
            forcedAnchorMatches.append({detectionIndex, catalogIndex, anchorDistance});
            matchedDetections.insert(detectionIndex);
            matchedCatalogStars.insert(catalogIndex);
            return true;
        };

        bool forcedAnchorsValid = appendForcedAnchor(candidate.anchorDetectionIndex, candidate.anchorCatalogIndex);
        if (candidate.sparseGuidedPair || candidate.guidedTriangle) {
            forcedAnchorsValid = forcedAnchorsValid
                && appendForcedAnchor(candidate.secondaryAnchorDetectionIndex, candidate.secondaryAnchorCatalogIndex);
        }
        if (candidate.guidedTriangle) {
            forcedAnchorsValid = forcedAnchorsValid
                && appendForcedAnchor(candidate.tertiaryAnchorDetectionIndex, candidate.tertiaryAnchorCatalogIndex);
        }

        if (forcedAnchorsValid)
        {
            const QVector<Match> automaticMatches = buildMatches(
                catalogContext,
                starDetections,
                detectionIndices,
                finalPass.projectedStars,
                finalMatchRadius);
            allMatches.reserve(automaticMatches.size() + allMatches.size());
            for (const Match& match : automaticMatches)
            {
                if (matchedDetections.contains(match.detectionIndex)
                    || matchedCatalogStars.contains(match.catalogIndex))
                {
                    continue;
                }
                matchedDetections.insert(match.detectionIndex);
                matchedCatalogStars.insert(match.catalogIndex);
                allMatches.append(match);
            }
        }
        else
        {
            allMatches.clear();
            rejectAnchoredCandidate = true;
        }
    }

    if (rejectAnchoredCandidate) {
        return finalPass;
    }

    if (allMatches.isEmpty())
    {
        allMatches = buildMatches(
            catalogContext,
            starDetections,
            detectionIndices,
            finalPass.projectedStars,
            finalMatchRadius);
    }
    finalPass.rawMatchCount = allMatches.size();
    if ((candidate.sparseGuidedPair || candidate.guidedTriangle)
        && (forcedAnchorMatches.size() >= (candidate.guidedTriangle ? 3 : 2)))
    {
        double anchorSquaredError = 0.0;
        int namedAnchorMatches = 0;
        QVector<Match> namedAnchorMatchesForBrightness;
        namedAnchorMatchesForBrightness.reserve(forcedAnchorMatches.size());
        for (const Match& match : forcedAnchorMatches)
        {
            if ((match.catalogIndex < 0)
                || (match.catalogIndex >= catalogContext.catalogStars.size())
                || (match.detectionIndex < 0)
                || (match.detectionIndex >= starDetections.size())
                || !isNamedSparseGuidedCatalogStar(catalogContext.catalogStars[match.catalogIndex])
                || !isStrongSparseGuidedDetection(starDetections[match.detectionIndex]))
            {
                continue;
            }

            anchorSquaredError += match.distancePixels * match.distancePixels;
            ++namedAnchorMatches;
            namedAnchorMatchesForBrightness.append(match);
        }

        finalPass.sparseGuidedNamedAnchorMatches = namedAnchorMatches;
        if (namedAnchorMatches > 0)
        {
            finalPass.sparseGuidedAnchorRmsErrorPixels =
                std::sqrt(anchorSquaredError / static_cast<double>(namedAnchorMatches));
            finalPass.sparseGuidedAnchorBrightnessRankError = matchBrightnessRankError(
                starDetections,
                detectionIndices,
                finalPass.projectedStars,
                namedAnchorMatchesForBrightness);
            finalPass.sparseGuidedAnchorMeanMagnitude = meanCatalogMagnitudeForMatches(
                catalogContext.catalogStars,
                namedAnchorMatchesForBrightness);
        }
    }

    finalPass.finalMatches = rejectOutlierMatches(
        allMatches,
        settings.m_plateSolveMinMatches,
        finalMatchRadius,
        &finalPass.outlierCount);
    if (candidate.anchored)
    {
        auto preserveForcedAnchorMatch = [&](int detectionIndex, int catalogIndex) {
            if ((detectionIndex < 0) || (catalogIndex < 0)) {
                return;
            }
            for (const Match& match : finalPass.finalMatches)
            {
                if ((match.detectionIndex == detectionIndex) || (match.catalogIndex == catalogIndex)) {
                    return;
                }
            }
            for (const Match& match : allMatches)
            {
                if ((match.detectionIndex == detectionIndex) && (match.catalogIndex == catalogIndex))
                {
                    finalPass.finalMatches.append(match);
                    return;
                }
            }
        };
        preserveForcedAnchorMatch(candidate.anchorDetectionIndex, candidate.anchorCatalogIndex);
        if (candidate.sparseGuidedPair || candidate.guidedTriangle) {
            preserveForcedAnchorMatch(candidate.secondaryAnchorDetectionIndex, candidate.secondaryAnchorCatalogIndex);
        }
        if (candidate.guidedTriangle) {
            preserveForcedAnchorMatch(candidate.tertiaryAnchorDetectionIndex, candidate.tertiaryAnchorCatalogIndex);
        }
    }

    const bool useNarrowGuidedBrightPrior = m_useDirectionSeedPreference && (isNarrowField(settings));
    appendSupplementalMatches(
        starDetections,
        finalPass.projectedStars,
        finalMatchRadius,
        restrictSupplementalMatchesToDetectionIndices ? &detectionIndices : nullptr,
        useNarrowGuidedBrightPrior,
        false,   // tight-artifact coincidence is applied label-only post-acceptance, not during scoring
        finalPass.finalMatches);
    appendWideBrightSupplementalMatches(
        settings,
        starDetections,
        finalPass.projectedStars,
        imageSize,
        finalMatchRadius,
        finalPass.finalMatches);

    if (!finalPass.finalMatches.isEmpty())
    {
        finalPass.brightnessRankError = matchBrightnessRankError(
            starDetections,
            detectionIndices,
            finalPass.projectedStars,
            finalPass.finalMatches);
        finalPass.meanCatalogMagnitude = meanCatalogMagnitudeForMatches(
            catalogContext.catalogStars,
            finalPass.finalMatches);
        double namedBrightAnchorSquaredError = 0.0;
        double namedBrightAnchorMagnitudeSum = 0.0;
        QSet<int> namedBrightAnchorCatalogs;
        QSet<int> namedBrightAnchorDetections;
        namedBrightAnchorCatalogs.reserve(finalPass.finalMatches.size());
        namedBrightAnchorDetections.reserve(finalPass.finalMatches.size());
        for (const Match& match : finalPass.finalMatches)
        {
            if ((match.catalogIndex < 0)
                || (match.catalogIndex >= catalogContext.catalogStars.size())
                || (match.detectionIndex < 0)
                || (match.detectionIndex >= starDetections.size())
                || namedBrightAnchorCatalogs.contains(match.catalogIndex)
                || namedBrightAnchorDetections.contains(match.detectionIndex))
            {
                continue;
            }

            const CatalogStar& catalogStar = catalogContext.catalogStars[match.catalogIndex];
            if (!isNamedSparseGuidedCatalogStar(catalogStar)
                || !isStrongSparseGuidedDetection(starDetections[match.detectionIndex])
                || (match.distancePixels > finalMatchRadius))
            {
                continue;
            }

            namedBrightAnchorCatalogs.insert(match.catalogIndex);
            namedBrightAnchorDetections.insert(match.detectionIndex);
            namedBrightAnchorSquaredError += match.distancePixels * match.distancePixels;
            namedBrightAnchorMagnitudeSum += catalogStar.magnitude;
        }
        finalPass.namedBrightAnchorMatches = namedBrightAnchorCatalogs.size();
        if (finalPass.namedBrightAnchorMatches > 0)
        {
            finalPass.namedBrightAnchorRmsErrorPixels =
                std::sqrt(namedBrightAnchorSquaredError / static_cast<double>(finalPass.namedBrightAnchorMatches));
            finalPass.namedBrightAnchorMeanMagnitude =
                namedBrightAnchorMagnitudeSum / static_cast<double>(finalPass.namedBrightAnchorMatches);
        }

        QSet<int> matchedCatalogIndices;
        matchedCatalogIndices.reserve(finalPass.finalMatches.size());
        QSet<int> matchedDetectionIndices;
        matchedDetectionIndices.reserve(finalPass.finalMatches.size());
        QHash<int, double> matchedDistanceByCatalog;
        matchedDistanceByCatalog.reserve(finalPass.finalMatches.size());
        QHash<int, int> matchedDetectionByCatalog;
        matchedDetectionByCatalog.reserve(finalPass.finalMatches.size());
        const double priorityMagnitudeLimit = useNarrowGuidedBrightPrior
            ? settings.m_plateSolveMaxMagnitude
            : 5.0;
        const bool useSeedRadialPrior = useNarrowGuidedBrightPrior && !m_directionSeedHasRollPreference;
        // P-A: candidate-independent -- served from the per-solve seed cache instead of rebuilt here.
        const SkyProjector& seedRadialProjector = seedCache.seedRadialProjector;
        const QPointF& seedRadialCenter = seedCache.seedRadialCenter;
        double prioritySeedRadialWeightedError = 0.0;
        double prioritySeedRadialWeight = 0.0;
        for (const Match& match : finalPass.finalMatches)
        {
            matchedCatalogIndices.insert(match.catalogIndex);
            matchedDetectionIndices.insert(match.detectionIndex);
            if ((match.catalogIndex < 0)
                || (match.catalogIndex >= catalogContext.catalogStars.size())
                || (match.detectionIndex < 0)
                || (match.detectionIndex >= starDetections.size()))
            {
                continue;
            }

            const CameraPipelineStarDetection& detection = starDetections[match.detectionIndex];
            const double magnitude = catalogContext.catalogStars[match.catalogIndex].magnitude;
            const auto existingDistanceIt = matchedDistanceByCatalog.constFind(match.catalogIndex);
            if ((existingDistanceIt == matchedDistanceByCatalog.cend())
                || (match.distancePixels < existingDistanceIt.value()))
            {
                matchedDistanceByCatalog.insert(match.catalogIndex, match.distancePixels);
                matchedDetectionByCatalog.insert(match.catalogIndex, match.detectionIndex);
            }
            const double magnitudeSupport =
                catalogMagnitudeSupportWeight(magnitude)
                * matchDistanceSupportWeight(match.distancePixels, finalMatchRadius);
            finalPass.magnitudeWeightedSupport += magnitudeSupport;
            if (magnitude <= priorityMagnitudeLimit) {
                finalPass.priorityMagnitudeWeightedSupport += magnitudeSupport;
                if (seedRadialProjector.valid)
                {
                    const auto visibleIt = catalogContext.visibleStarIndexByCatalogIndex.constFind(match.catalogIndex);
                    if (visibleIt != catalogContext.visibleStarIndexByCatalogIndex.cend())
                    {
                        const int visibleIndex = visibleIt.value();
                        if ((visibleIndex >= 0) && (visibleIndex < catalogContext.visibleStars.size()))
                        {
                            // P-A: seed projection served from the cache (bit-identical value).
                            if (seedCache.seedPointValid.value(visibleIndex, 0))
                            {
                                const QPointF seedPoint = seedCache.seedPoints[visibleIndex];
                                const double seedRadius = pointDistancePixels(seedPoint, seedRadialCenter);
                                const double detectionRadius = pointDistancePixels(detection.m_center, seedRadialCenter);
                                const double radialError = std::fabs(detectionRadius - seedRadius);
                                const double radialWeight = catalogMagnitudeSupportWeight(magnitude);
                                prioritySeedRadialWeightedError += radialWeight * radialError;
                                prioritySeedRadialWeight += radialWeight;
                                ++finalPass.prioritySeedRadialChecks;
                            }
                        }
                    }
                }
            }

            if (magnitude <= 9.5)
            {
                ++finalPass.brightCatalogShapeChecks;
                if (isImplausiblyCompactBrightCatalogDetection(detection, magnitude, useNarrowGuidedBrightPrior))
                {
                    ++finalPass.brightCatalogShapeMismatches;
                }
            }
        }
        if (prioritySeedRadialWeight > 0.0) {
            finalPass.prioritySeedRadialErrorPixels = prioritySeedRadialWeightedError / prioritySeedRadialWeight;
        }
        if (seedRadialProjector.valid)
        {
            const double seedMatchRadius = std::max(
                static_cast<double>(finalMatchRadius) * 2.0,
                48.0);
            const double maxImageRadius = std::hypot(
                std::max(seedRadialCenter.x(), static_cast<double>(imageSize.width()) - seedRadialCenter.x()),
                std::max(seedRadialCenter.y(), static_cast<double>(imageSize.height()) - seedRadialCenter.y()))
                + seedMatchRadius;
            // P-A: sorted detection radii served from the per-solve cache (bit-identical).
            const QVector<double>& detectionRadii = seedCache.sortedDetectionRadii;
            const auto hasDetectionAtSeedRadius = [&detectionRadii, seedMatchRadius](double seedRadius) {
                const auto lower = std::lower_bound(
                    detectionRadii.cbegin(),
                    detectionRadii.cend(),
                    seedRadius - seedMatchRadius);
                return (lower != detectionRadii.cend()) && (*lower <= (seedRadius + seedMatchRadius));
            };

            for (int visibleStarIndex = 0; visibleStarIndex < catalogContext.visibleStars.size(); ++visibleStarIndex)
            {
                const VisibleCatalogStar& visibleStar = catalogContext.visibleStars[visibleStarIndex];
                if ((visibleStar.catalogIndex < 0)
                    || (visibleStar.catalogIndex >= catalogContext.catalogStars.size()))
                {
                    continue;
                }

                const CatalogStar& catalogStar = catalogContext.catalogStars[visibleStar.catalogIndex];
                if (catalogStar.magnitude > priorityMagnitudeLimit) {
                    continue;
                }

                // P-A: seed projection served from the cache (bit-identical value).
                if (!seedCache.seedPointValid.value(visibleStarIndex, 0)) {
                    continue;
                }
                const QPointF seedPoint = seedCache.seedPoints[visibleStarIndex];

                const double seedRadius = pointDistancePixels(seedPoint, seedRadialCenter);
                if ((seedRadius > maxImageRadius) || !hasDetectionAtSeedRadius(seedRadius)) {
                    continue;
                }

                const double seedRadialWeight = catalogMagnitudeSupportWeight(catalogStar.magnitude);
                finalPass.seedRadialMagnitudeSupport += seedRadialWeight;
                ++finalPass.seedRadialCatalogStars;

                const auto matchedDetectionIt = matchedDetectionByCatalog.constFind(visibleStar.catalogIndex);
                if (matchedDetectionIt == matchedDetectionByCatalog.cend()) {
                    continue;
                }

                const int matchedDetectionIndex = matchedDetectionIt.value();
                if ((matchedDetectionIndex < 0) || (matchedDetectionIndex >= starDetections.size())) {
                    continue;
                }

                const double detectionRadius = pointDistancePixels(
                    starDetections[matchedDetectionIndex].m_center,
                    seedRadialCenter);
                const double radialDistanceWeight = matchDistanceSupportWeight(
                    std::fabs(detectionRadius - seedRadius),
                    seedMatchRadius);
                const auto matchedDistanceIt = matchedDistanceByCatalog.constFind(visibleStar.catalogIndex);
                const double finalDistanceWeight = (matchedDistanceIt == matchedDistanceByCatalog.cend())
                    ? 1.0
                    : matchDistanceSupportWeight(matchedDistanceIt.value(), finalMatchRadius);
                finalPass.matchedSeedRadialMagnitudeSupport +=
                    seedRadialWeight * radialDistanceWeight * finalDistanceWeight;
                ++finalPass.matchedSeedRadialCatalogStars;
            }

            finalPass.seedRadialMagnitudeMatchFraction = (finalPass.seedRadialMagnitudeSupport > 0.0)
                ? finalPass.matchedSeedRadialMagnitudeSupport / finalPass.seedRadialMagnitudeSupport
                : 1.0;
        }
        // P-A: for the narrow-guided prior this list is all-detections (caller-independent) and is
        // served from the per-solve cache; otherwise it derives from the caller's detectionIndices
        // and is built inline (bit-identical to before). const& so any later mutation fails to build.
        QVector<int> brightDetectionIndicesLocal;
        if (!useNarrowGuidedBrightPrior)
        {
            brightDetectionIndicesLocal = detectionIndices;
            brightDetectionIndicesLocal.erase(
                std::remove_if(
                    brightDetectionIndicesLocal.begin(),
                    brightDetectionIndicesLocal.end(),
                    [&starDetections](int detectionIndex) {
                        return (detectionIndex < 0)
                            || (detectionIndex >= starDetections.size())
                            || !isDetectionUsableForBrightPrior(starDetections[detectionIndex]);
                    }),
                brightDetectionIndicesLocal.end());
            std::sort(brightDetectionIndicesLocal.begin(), brightDetectionIndicesLocal.end(), [this, &starDetections](int lhs, int rhs) {
                const double lhsBrightness = cachedDetectionBrightnessMetric(starDetections, lhs);
                const double rhsBrightness = cachedDetectionBrightnessMetric(starDetections, rhs);
                const double lhsReliability = cachedDetectionReliabilityMetric(starDetections, lhs);
                const double rhsReliability = cachedDetectionReliabilityMetric(starDetections, rhs);
                const double lhsImportance = lhsBrightness * std::sqrt(std::max(0.0, lhsReliability));
                const double rhsImportance = rhsBrightness * std::sqrt(std::max(0.0, rhsReliability));
                if (!qFuzzyCompare(lhsImportance + 1.0, rhsImportance + 1.0)) {
                    return lhsImportance > rhsImportance;
                }
                return lhsBrightness > rhsBrightness;
            });
        }
        const QVector<int>& brightDetectionIndices = useNarrowGuidedBrightPrior
            ? seedCache.brightDetectionIndices
            : brightDetectionIndicesLocal;
        const int brightDetectionLimit = std::min(
            useNarrowGuidedBrightPrior ? 24 : 8,
            static_cast<int>(brightDetectionIndices.size()));
        QHash<int, int> matchedCatalogByDetection;
        matchedCatalogByDetection.reserve(finalPass.finalMatches.size());
        for (const Match& match : finalPass.finalMatches) {
            matchedCatalogByDetection.insert(match.detectionIndex, match.catalogIndex);
        }
        double brightMagnitudePenalty = 0.0;
        double brightMagnitudeWeight = 0.0;
        for (int i = 0; i < brightDetectionLimit; ++i)
        {
            const int detectionIndex = brightDetectionIndices[i];
            ++finalPass.brightDetections;
            if (matchedDetectionIndices.contains(detectionIndex)) {
                ++finalPass.matchedBrightDetections;
            }
            const double rankWeight = (i == 0) ? 4.0 : (i < 3) ? 2.0 : 1.0;
            const auto catalogIt = matchedCatalogByDetection.constFind(detectionIndex);
            if (catalogIt == matchedCatalogByDetection.cend())
            {
                brightMagnitudePenalty += rankWeight * ((i == 0) ? 3.0 : (i < 3) ? 2.0 : 1.0);
                brightMagnitudeWeight += rankWeight;
                continue;
            }
            const int catalogIndex = catalogIt.value();
            const double catalogMagnitude = ((catalogIndex >= 0) && (catalogIndex < catalogContext.catalogStars.size()))
                ? catalogContext.catalogStars[catalogIndex].magnitude
                : std::numeric_limits<double>::infinity();
            const double expectedMaxMagnitude = m_useDirectionSeedPreference && (isNarrowField(settings))
                ? ((i == 0) ? 8.5 : (i < 3) ? 10.0 : 11.5)
                : ((i == 0) ? 1.0 : (i < 3) ? 3.0 : 4.5);
            brightMagnitudePenalty += rankWeight * std::max(0.0, catalogMagnitude - expectedMaxMagnitude);
            brightMagnitudeWeight += rankWeight;
        }
        finalPass.brightDetectionMatchFraction = (finalPass.brightDetections > 0)
            ? static_cast<double>(finalPass.matchedBrightDetections) / static_cast<double>(finalPass.brightDetections)
            : 1.0;
        finalPass.brightDetectionMagnitudeError = (brightMagnitudeWeight > 0.0)
            ? brightMagnitudePenalty / brightMagnitudeWeight
            : 0.0;

        if (useNarrowGuidedBrightPrior)
        {
            const SkyProjector seedProjector = createProjector(
                settings,
                imageSize,
                m_directionSeedReferenceAzimuthDegrees,
                m_directionSeedReferenceElevationDegrees,
                m_directionSeedReferenceRollDegrees,
                m_directionSeedReferenceFovDegrees,
                settings.m_lensCenterOffsetX,
                settings.m_lensCenterOffsetY,
                settings.m_lensDistortionK1);
            const double seedMatchRadius = std::max(
                static_cast<double>(finalMatchRadius) * 2.0,
                48.0);
            const QRectF seedBrightBounds(
                -seedMatchRadius,
                -seedMatchRadius,
                imageSize.width() + 2.0 * seedMatchRadius,
                imageSize.height() + 2.0 * seedMatchRadius);
            if (seedProjector.valid)
            {
                double prioritySeedProjectedWeightedError = 0.0;
                double prioritySeedProjectedWeight = 0.0;
                for (const VisibleCatalogStar& visibleStar : catalogContext.visibleStars)
                {
                    if ((visibleStar.catalogIndex < 0)
                        || (visibleStar.catalogIndex >= catalogContext.catalogStars.size()))
                    {
                        continue;
                    }

                    const CatalogStar& catalogStar = catalogContext.catalogStars[visibleStar.catalogIndex];
                    if (catalogStar.magnitude > priorityMagnitudeLimit) {
                        continue;
                    }

                    QPointF seedPoint;
                    if (!projectVector(seedProjector, visibleStar.vector, seedPoint)
                        || !seedBrightBounds.contains(seedPoint))
                    {
                        continue;
                    }

                    bool hasNearbySeedDetection = false;
                    for (const CameraPipelineStarDetection& detection : starDetections)
                    {
                        if (pointDistancePixels(detection.m_center, seedPoint) <= seedMatchRadius)
                        {
                            hasNearbySeedDetection = true;
                            break;
                        }
                    }
                    if (!hasNearbySeedDetection) {
                        continue;
                    }

                    const bool isSeedBrightStar = catalogStar.magnitude <= 10.5;
                    const double seedProjectedWeight = catalogMagnitudeSupportWeight(catalogStar.magnitude);
                    finalPass.seedProjectedMagnitudeSupport += seedProjectedWeight;
                    if (isSeedBrightStar) {
                        ++finalPass.seedProjectedBrightStars;
                    }
                    for (const Match& match : finalPass.finalMatches)
                    {
                        if ((match.catalogIndex != visibleStar.catalogIndex)
                            || (match.detectionIndex < 0)
                            || (match.detectionIndex >= starDetections.size()))
                        {
                            continue;
                        }

                        if (pointDistancePixels(starDetections[match.detectionIndex].m_center, seedPoint) <= seedMatchRadius)
                        {
                            const double seedProjectedError = pointDistancePixels(
                                starDetections[match.detectionIndex].m_center,
                                seedPoint);
                            const double matchedDistanceWeight =
                                matchDistanceSupportWeight(match.distancePixels, finalMatchRadius);
                            finalPass.matchedSeedProjectedMagnitudeSupport +=
                                seedProjectedWeight * matchedDistanceWeight;
                            prioritySeedProjectedWeightedError += seedProjectedWeight * seedProjectedError;
                            prioritySeedProjectedWeight += seedProjectedWeight;
                            ++finalPass.prioritySeedProjectedChecks;
                            if (isSeedBrightStar) {
                                ++finalPass.matchedSeedProjectedBrightStars;
                            }
                            break;
                        }
                    }
                }
                if (prioritySeedProjectedWeight > 0.0) {
                    finalPass.prioritySeedProjectedErrorPixels =
                        prioritySeedProjectedWeightedError / prioritySeedProjectedWeight;
                }
            }
            finalPass.seedProjectedBrightMatchFraction = (finalPass.seedProjectedBrightStars > 0)
                ? static_cast<double>(finalPass.matchedSeedProjectedBrightStars) / static_cast<double>(finalPass.seedProjectedBrightStars)
                : 1.0;
            finalPass.seedProjectedMagnitudeMatchFraction = (finalPass.seedProjectedMagnitudeSupport > 0.0)
                ? finalPass.matchedSeedProjectedMagnitudeSupport / finalPass.seedProjectedMagnitudeSupport
                : 1.0;
        }

        const QRectF projectedMagnitudeBounds(
            -finalMatchRadius,
            -finalMatchRadius,
            imageSize.width() + 2.0 * finalMatchRadius,
            imageSize.height() + 2.0 * finalMatchRadius);
        if (useNarrowGuidedBrightPrior)
        {
            for (const ProjectedCatalogStar& projectedStar : finalPass.projectedStars)
            {
                if (!projectedMagnitudeBounds.contains(projectedStar.point)
                    || !std::isfinite(projectedStar.magnitude))
                {
                    continue;
                }

                const double projectedWeight = catalogMagnitudeSupportWeight(projectedStar.magnitude);
                finalPass.projectedMagnitudeSupport += projectedWeight;
                const auto matchedDistanceIt = matchedDistanceByCatalog.constFind(projectedStar.catalogIndex);
                if (matchedDistanceIt != matchedDistanceByCatalog.cend())
                {
                    finalPass.matchedProjectedMagnitudeSupport +=
                        projectedWeight * matchDistanceSupportWeight(matchedDistanceIt.value(), finalMatchRadius);
                }
            }
            finalPass.projectedMagnitudeMatchFraction = (finalPass.projectedMagnitudeSupport > 0.0)
                ? finalPass.matchedProjectedMagnitudeSupport / finalPass.projectedMagnitudeSupport
                : 1.0;
        }

        const QRectF brightProjectedBounds(
            -finalMatchRadius,
            -finalMatchRadius,
            imageSize.width() + 2.0 * finalMatchRadius,
            imageSize.height() + 2.0 * finalMatchRadius);
        const bool useNarrowGuidedBrightProjectedPrior = m_useDirectionSeedPreference && (isNarrowField(settings));
        const double brightProjectedMagnitudeLimit = useNarrowGuidedBrightProjectedPrior
            ? std::min(settings.m_plateSolveMaxMagnitude, 13.0)
            : 5.0;
        QVector<ProjectedCatalogStar> brightProjectedStars;
        brightProjectedStars.reserve(16);
        for (const ProjectedCatalogStar& projectedStar : finalPass.projectedStars)
        {
            if ((projectedStar.magnitude > brightProjectedMagnitudeLimit)
                || !brightProjectedBounds.contains(projectedStar.point))
            {
                continue;
            }
            brightProjectedStars.append(projectedStar);
        }
        std::sort(brightProjectedStars.begin(), brightProjectedStars.end(), [](const ProjectedCatalogStar& lhs, const ProjectedCatalogStar& rhs) {
            return lhs.magnitude < rhs.magnitude;
        });
        const int brightProjectedLimit = useNarrowGuidedBrightProjectedPrior ? 12 : 8;
        if (brightProjectedStars.size() > brightProjectedLimit) {
            brightProjectedStars.resize(brightProjectedLimit);
        }
        for (const ProjectedCatalogStar& projectedStar : brightProjectedStars)
        {
            ++finalPass.brightProjectedStars;
            if (matchedCatalogIndices.contains(projectedStar.catalogIndex)) {
                ++finalPass.matchedBrightProjectedStars;
            }
        }
        if (qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_BRIGHTPROJ")
            && useNarrowGuidedBrightProjectedPrior
            && (finalPass.finalMatches.size() >= 150))
        {
            for (const ProjectedCatalogStar& projectedStar : brightProjectedStars)
            {
                const bool matched = matchedCatalogIndices.contains(projectedStar.catalogIndex);
                double nearestDist = std::numeric_limits<double>::infinity();
                int nearestDetection = -1;
                for (int di = 0; di < starDetections.size(); ++di) {
                    const double d = pointDistancePixels(starDetections[di].m_center, projectedStar.point);
                    if (d < nearestDist) { nearestDist = d; nearestDetection = di; }
                }
                const QString name = ((projectedStar.catalogIndex >= 0)
                    && (projectedStar.catalogIndex < catalogContext.catalogStars.size()))
                    ? catalogDisplayName(catalogContext.catalogStars[projectedStar.catalogIndex])
                    : QStringLiteral("?");
                qCDebug(cameraPlateSolverLog).noquote().nospace()
                    << "BRIGHTPROJ"
                    << " Az=" << candidate.azimuthDegrees
                    << " El=" << candidate.elevationDegrees
                    << " Roll=" << candidate.rollDegrees
                    << " matches=" << finalPass.finalMatches.size()
                    << " name=" << name
                    << " mag=" << projectedStar.magnitude
                    << " point=(" << projectedStar.point.x() << "," << projectedStar.point.y() << ")"
                    << " matched=" << matched
                    << " nearestDet=" << nearestDetection
                    << " nearestDist=" << nearestDist
                    << " nearestPeak=" << ((nearestDetection >= 0) ? starDetections[nearestDetection].m_peakValue : -1.0f)
                    << " nearestSat=" << ((nearestDetection >= 0) ? starDetections[nearestDetection].m_saturated : false);
            }
        }
        finalPass.brightProjectedMatchFraction = (finalPass.brightProjectedStars > 0)
            ? static_cast<double>(finalPass.matchedBrightProjectedStars) / static_cast<double>(finalPass.brightProjectedStars)
            : 1.0;
        double maxError = 0.0;
        for (const Match& match : finalPass.finalMatches)
        {
            maxError = std::max(maxError, match.distancePixels);
        }
        finalPass.rmsErrorPixels = weightedRmsDistancePixels(starDetections, finalPass.finalMatches);
        finalPass.medianErrorPixels = medianDistancePixels(finalPass.finalMatches);
        finalPass.maxErrorPixels = maxError;
        finalPass.pose.meanCatalogMagnitude = finalPass.meanCatalogMagnitude;
    }

    // P-C: memoize the derived gate/score values that the ranking comparator would otherwise
    // recompute for both sides on every pairwise comparison. Pure functions of (settings, this
    // pass); computed once here now that finalPass is fully populated.
    finalPass.cachedStrongDenseNarrowGuided = hasStrongDenseNarrowGuidedFinalPass(settings, finalPass);
    finalPass.cachedFinalMatchPassScore = finalMatchPassScore(settings, finalPass);
    finalPass.cachedNarrowGuidedBrightConsistencyScore = narrowGuidedBrightConsistencyScore(settings, finalPass);
    finalPass.cachedNarrowGuidedSeedConsistencyScore = narrowGuidedSeedConsistencyScore(settings, finalPass);
    finalPass.cachedGatesValid = true;

    // Verifier-study instrumentation (Tier 1): dump one machine-readable line per final-pass
    // candidate so an OFFLINE study can label each pose right/wrong against the corpus truth and
    // evaluate discriminator features (excess-over-chance at a radius ladder, brightness agreement,
    // residual structure) across every failure class at once. Raw quantities only — features are
    // engineered offline. Off by default; enabling changes logging only, never behaviour.
    static const bool candidateDumpEnabled =
        qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_CANDIDATE_DUMP");
    if (candidateDumpEnabled && finalPass.projectorValid)
    {
        // Match-tightness ladder: counts within fixed absolute radii. Chance matches distribute
        // with density ~r (CDF ~r^2), correct matches concentrate at small r, so the ladder is
        // what lets the offline study compute excess-over-chance at the informative radius
        // (a single-radius count saturates in dense fields for right AND wrong poses alike).
        int within2 = 0, within4 = 0, within8 = 0, within16 = 0;
        for (const Match& match : finalPass.finalMatches)
        {
            if (match.distancePixels <= 2.0) ++within2;
            if (match.distancePixels <= 4.0) ++within4;
            if (match.distancePixels <= 8.0) ++within8;
            if (match.distancePixels <= 16.0) ++within16;
        }
        qCDebug(cameraPlateSolverLog).noquote().nospace()
            << "CandidateDump,az=" << QString::number(finalPass.pose.azimuthDegrees, 'f', 3)
            << ",el=" << QString::number(finalPass.pose.elevationDegrees, 'f', 3)
            << ",roll=" << QString::number(finalPass.pose.rollDegrees, 'f', 3)
            << ",fov=" << QString::number(finalPass.pose.fovDegrees, 'f', 4)
            << ",cx=" << QString::number(finalPass.pose.centerOffsetXPixels, 'f', 1)
            << ",cy=" << QString::number(finalPass.pose.centerOffsetYPixels, 'f', 1)
            << ",k1=" << QString::number(finalPass.pose.distortionK1, 'f', 4)
            << ",D=" << detectionIndices.size()
            << ",C=" << finalPass.projectedStars.size()
            << ",M=" << finalPass.finalMatches.size()
            << ",m2=" << within2 << ",m4=" << within4 << ",m8=" << within8 << ",m16=" << within16
            << ",rms=" << QString::number(finalPass.rmsErrorPixels, 'f', 2)
            << ",med=" << QString::number(finalPass.medianErrorPixels, 'f', 2)
            << ",max=" << QString::number(finalPass.maxErrorPixels, 'f', 2)
            << ",r=" << QString::number(finalMatchRadius, 'f', 1)
            << ",W=" << imageSize.width() << ",H=" << imageSize.height()
            << ",bD=" << finalPass.brightDetections << ",mBD=" << finalPass.matchedBrightDetections
            << ",bP=" << finalPass.brightProjectedStars << ",mBP=" << finalPass.matchedBrightProjectedStars
            << ",magErr=" << QString::number(finalPass.brightDetectionMagnitudeError, 'f', 3)
            << ",rank=" << (std::isfinite(finalPass.brightnessRankError)
                ? QString::number(finalPass.brightnessRankError, 'f', 3) : QStringLiteral("inf"))
            << [&]() -> QString {
                // v2: per-match residual VECTORS for finalist-grade candidates, so the offline
                // study can compute residual-FIELD coherence — the one feature with a physical
                // reason to separate all-sky rotations (true-pose residuals are a smooth function
                // of image position: lens-model error + slight rotation; coincidence residuals are
                // not). Subsampled to <=48 matches; x:y = detection position, dx:dy = detection
                // minus projected catalogue position.
                if (finalPass.finalMatches.size() < 6) {
                    return QString();
                }
                QHash<int, QPointF> projectedByCatalog;
                projectedByCatalog.reserve(finalPass.projectedStars.size());
                for (const ProjectedCatalogStar& star : finalPass.projectedStars) {
                    projectedByCatalog.insert(star.catalogIndex, star.point);
                }
                QString residuals = QStringLiteral(",res=");
                const int matchCount = static_cast<int>(finalPass.finalMatches.size());
                const int stride = std::max(1, matchCount / 48);
                bool first = true;
                for (int i = 0; i < matchCount; i += stride)
                {
                    const Match& match = finalPass.finalMatches[i];
                    if ((match.detectionIndex < 0) || (match.detectionIndex >= starDetections.size())
                        || !projectedByCatalog.contains(match.catalogIndex))
                    {
                        continue;
                    }
                    const QPointF detection = starDetections[match.detectionIndex].m_center;
                    const QPointF projected = projectedByCatalog.value(match.catalogIndex);
                    if (!first) {
                        residuals += QLatin1Char(';');
                    }
                    first = false;
                    residuals += QStringLiteral("%1:%2:%3:%4")
                        .arg(detection.x(), 0, 'f', 0)
                        .arg(detection.y(), 0, 'f', 0)
                        .arg(detection.x() - projected.x(), 0, 'f', 2)
                        .arg(detection.y() - projected.y(), 0, 'f', 2);
                }
                return residuals;
            }();
    }

    return finalPass;
}

void CameraPlateSolver::SolverContext::logFinalMatchPassEvaluation(const char *stage, const FinalMatchPassEvaluation& evaluation, bool best)
{
    if (!evaluation.projectorValid) {
        return;
    }

    qCDebug(cameraPlateSolverLog).noquote().nospace()
        << "CameraPlateSolver[" << stage << "] "
        << (best ? "best " : "candidate ")
        << "Az=" << evaluation.pose.azimuthDegrees
        << " El=" << evaluation.pose.elevationDegrees
        << " Roll=" << evaluation.pose.rollDegrees
        << " FoV=" << evaluation.pose.fovDegrees
        << " finalMatches=" << evaluation.finalMatches.size()
        << " rawMatches=" << evaluation.rawMatchCount
        << " RMS=" << evaluation.rmsErrorPixels
        << " Median=" << evaluation.medianErrorPixels
        << " Max=" << evaluation.maxErrorPixels
        << " BrightnessErr=" << evaluation.brightnessRankError
        << " MeanMag=" << evaluation.meanCatalogMagnitude
        << " BrightDetections=" << evaluation.matchedBrightDetections << "/" << evaluation.brightDetections
        << " BrightMagErr=" << evaluation.brightDetectionMagnitudeError
        << " BrightProjected=" << evaluation.matchedBrightProjectedStars << "/" << evaluation.brightProjectedStars
        << " MagSupport=" << evaluation.magnitudeWeightedSupport
        << " PriorityMagSupport=" << evaluation.priorityMagnitudeWeightedSupport
        << " ProjectedMagSupport=" << evaluation.matchedProjectedMagnitudeSupport << "/" << evaluation.projectedMagnitudeSupport
        << " ProjectedMagFraction=" << evaluation.projectedMagnitudeMatchFraction
        << " SeedMagSupport=" << evaluation.matchedSeedProjectedMagnitudeSupport << "/" << evaluation.seedProjectedMagnitudeSupport
        << " SeedMagFraction=" << evaluation.seedProjectedMagnitudeMatchFraction
        << " SeedRadialMagSupport=" << evaluation.matchedSeedRadialMagnitudeSupport << "/" << evaluation.seedRadialMagnitudeSupport
        << " SeedRadialMagFraction=" << evaluation.seedRadialMagnitudeMatchFraction
        << " SeedRadialStars=" << evaluation.matchedSeedRadialCatalogStars << "/" << evaluation.seedRadialCatalogStars
        << " SeedRadial=" << evaluation.prioritySeedRadialErrorPixels << "/" << evaluation.prioritySeedRadialChecks
        << " SeedProjected=" << evaluation.prioritySeedProjectedErrorPixels << "/" << evaluation.prioritySeedProjectedChecks
        << " SeedBright=" << evaluation.matchedSeedProjectedBrightStars << "/" << evaluation.seedProjectedBrightStars
        << " BrightShape=" << evaluation.brightCatalogShapeMismatches << "/" << evaluation.brightCatalogShapeChecks
        << " NamedAnchors=" << evaluation.namedBrightAnchorMatches
        << " NamedAnchorRMS=" << evaluation.namedBrightAnchorRmsErrorPixels
        << " SparseAnchors=" << evaluation.sparseGuidedNamedAnchorMatches
        << " AnchorRMS=" << evaluation.sparseGuidedAnchorRmsErrorPixels
        << " AnchorBrightErr=" << evaluation.sparseGuidedAnchorBrightnessRankError
        << " projectedStars=" << evaluation.projectedStars.size()
        << " K1=" << evaluation.pose.distortionK1;
}

bool CameraPlateSolver::SolverContext::isBetterFinalPassEvaluation(const Evaluation& candidate, const Evaluation& best, int retainedMatchThreshold, bool useGuidedDirectionScoring)
{
    if (!candidate.valid) {
        return false;
    }
    if (!best.valid) {
        return true;
    }

    const bool candidateMeetsThreshold = candidate.matchCount >= retainedMatchThreshold;
    const bool bestMeetsThreshold = best.matchCount >= retainedMatchThreshold;
    if (candidateMeetsThreshold != bestMeetsThreshold) {
        return candidateMeetsThreshold;
    }

    if (candidateMeetsThreshold)
    {
        const double comparableRms = std::min(
            m_weakModeNormalizationPixels,
            kWideFovMagnitudePreferenceMaxRmsPixels);
        if (shouldPreferBrighterCatalogMean(
                candidate.meanCatalogMagnitude,
                best.meanCatalogMagnitude,
                candidate.rmsErrorPixels,
                best.rmsErrorPixels,
                comparableRms))
        {
            return true;
        }
        if (shouldPreferBrighterCatalogMean(
                best.meanCatalogMagnitude,
                candidate.meanCatalogMagnitude,
                best.rmsErrorPixels,
                candidate.rmsErrorPixels,
                comparableRms))
        {
            return false;
        }

        const double candidateMedian = medianDistancePixels(candidate.matches);
        const double bestMedian = medianDistancePixels(best.matches);
        if (!qFuzzyCompare(candidateMedian + 1.0, bestMedian + 1.0)) {
            return candidateMedian < bestMedian;
        }
        if (!qFuzzyCompare(candidate.rmsErrorPixels + 1.0, best.rmsErrorPixels + 1.0)) {
            return candidate.rmsErrorPixels < best.rmsErrorPixels;
        }
        if (!qFuzzyCompare(candidate.brightnessRankError + 1.0, best.brightnessRankError + 1.0)) {
            return candidate.brightnessRankError < best.brightnessRankError;
        }
        if (candidate.matchCount != best.matchCount) {
            return candidate.matchCount > best.matchCount;
        }
        return useGuidedDirectionScoring
            ? isBetterGuidedDirectionEvaluation(candidate, best)
            : isBetterEvaluation(candidate, best);
    }

    if (candidate.matchCount != best.matchCount) {
        return candidate.matchCount > best.matchCount;
    }
    if (!qFuzzyCompare(candidate.rmsErrorPixels + 1.0, best.rmsErrorPixels + 1.0)) {
        return candidate.rmsErrorPixels < best.rmsErrorPixels;
    }

    return useGuidedDirectionScoring
        ? isBetterGuidedDirectionEvaluation(candidate, best)
        : isBetterEvaluation(candidate, best);
}

int CameraPlateSolver::SolverContext::minimumRetainedMatchesForFinalPass(const Evaluation& reference, int minMatchCount)
{
    if (!reference.valid) {
        return std::max(3, minMatchCount);
    }

    // Keep the tightening pass honest: it should preserve most of the coarse/refined
    // correspondences, not collapse to a tiny high-confidence subset that hijacks the solve.
    const int relativeFloor = static_cast<int>(std::ceil(static_cast<double>(reference.matchCount) * 0.70));
    return std::min(reference.matchCount, std::max(minMatchCount, std::max(3, relativeFloor)));
}

double CameraPlateSolver::SolverContext::wideFinalPassMatchWeight(const CameraSettings& settings, int matchCount)
{
    const bool useNarrowGuidedMatchCap = m_useDirectionSeedPreference
        && (isNarrowField(settings));
    if (!m_useWideCatalogMagnitudePreference && !useNarrowGuidedMatchCap) {
        return static_cast<double>(matchCount);
    }

    const int usefulMatchCap = useNarrowGuidedMatchCap
        ? std::max(settings.m_plateSolveMinMatches + 14, 18)
        : std::max(settings.m_plateSolveMinMatches + 4, 8);
    const int cappedMatches = std::min(matchCount, usefulMatchCap);
    const int extraMatches = std::max(0, matchCount - usefulMatchCap);
    return static_cast<double>(cappedMatches) + 0.15 * std::log1p(static_cast<double>(extraMatches));
}

double CameraPlateSolver::SolverContext::wideFinalPassMagnitudeAffinity(const FinalMatchPassEvaluation& evaluation)
{
    if (!m_useWideCatalogMagnitudePreference
        || !std::isfinite(evaluation.meanCatalogMagnitude)
        || (evaluation.finalMatches.size() < 3))
    {
        return 1.0;
    }

    const double faintExcess = std::max(0.0, evaluation.meanCatalogMagnitude - 2.0);
    return 1.0 / (1.0 + 0.65 * faintExcess * faintExcess);
}

double CameraPlateSolver::SolverContext::brightDetectionCoverageAffinity(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation)
{
    const bool useBrightDetectionPreference = isWidePlateSolveContext(settings)
        || (m_useDirectionSeedPreference && (isNarrowField(settings)));
    if (!useBrightDetectionPreference
        || (evaluation.brightDetections < 3))
    {
        return 1.0;
    }

    const double coverage = std::clamp(evaluation.brightDetectionMatchFraction, 0.0, 1.0);
    return 0.20 + 0.80 * coverage * coverage;
}

double CameraPlateSolver::SolverContext::brightProjectedCoverageAffinity(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation)
{
    const bool useNarrowGuidedProjectedPreference = m_useDirectionSeedPreference
        && (isNarrowField(settings));
    const int minimumProjectedStars = useNarrowGuidedProjectedPreference ? 2 : 3;
    if ((!isWidePlateSolveContext(settings) && !useNarrowGuidedProjectedPreference)
        || (evaluation.brightProjectedStars < minimumProjectedStars))
    {
        return 1.0;
    }

    const double coverage = std::clamp(evaluation.brightProjectedMatchFraction, 0.0, 1.0);
    return (useNarrowGuidedProjectedPreference ? 0.05 : 0.10)
        + (useNarrowGuidedProjectedPreference ? 0.95 : 0.90) * coverage * coverage;
}

double CameraPlateSolver::SolverContext::brightDetectionMagnitudeAffinity(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation)
{
    const bool useNarrowGuidedMagnitudePrior = m_useDirectionSeedPreference
        && (isNarrowField(settings));
    if ((!isWidePlateSolveContext(settings) && !useNarrowGuidedMagnitudePrior)
        || (evaluation.brightDetections < 3))
    {
        return 1.0;
    }

    const double error = std::max(0.0, evaluation.brightDetectionMagnitudeError);
    const double strength = useNarrowGuidedMagnitudePrior ? 2.5 : 4.0;
    return 1.0 / (1.0 + strength * error * error);
}

double CameraPlateSolver::SolverContext::projectedMagnitudeCoverageAffinity(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation)
{
    if (!m_useDirectionSeedPreference
        || (!isNarrowField(settings))
        || (evaluation.projectedMagnitudeSupport <= 0.0))
    {
        return 1.0;
    }

    const double coverage = std::clamp(evaluation.projectedMagnitudeMatchFraction, 0.0, 1.0);
    return 0.10 + 0.90 * coverage * coverage;
}

double CameraPlateSolver::SolverContext::narrowGuidedBrightConsistencyScore(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation)
{
    if (!isLowMagnitudeNarrowGuidedSolve(settings) || !evaluation.projectorValid) {
        return 0.0;
    }

    const double detectionCoverage = (evaluation.brightDetections > 0)
        ? std::clamp(evaluation.brightDetectionMatchFraction, 0.0, 1.0)
        : 1.0;
    const double projectedCoverage = (evaluation.brightProjectedStars > 0)
        ? std::clamp(evaluation.brightProjectedMatchFraction, 0.0, 1.0)
        : 1.0;
    const double magnitudeAffinity = 1.0 / (1.0
        + 2.0 * std::max(0.0, evaluation.brightDetectionMagnitudeError)
            * std::max(0.0, evaluation.brightDetectionMagnitudeError));
    const double projectedMagnitudeCoverage = (evaluation.projectedMagnitudeSupport > 0.0)
        ? std::clamp(evaluation.projectedMagnitudeMatchFraction, 0.0, 1.0)
        : 1.0;
    const double shapeAffinity = (evaluation.brightCatalogShapeChecks > 0)
        ? 1.0 - std::clamp(
            static_cast<double>(evaluation.brightCatalogShapeMismatches)
                / static_cast<double>(evaluation.brightCatalogShapeChecks),
            0.0,
            1.0)
        : 1.0;

    return 4.0 * detectionCoverage
        + 5.0 * projectedCoverage
        + 5.0 * projectedMagnitudeCoverage
        + 3.0 * magnitudeAffinity
        + 2.0 * shapeAffinity
        + 0.20 * static_cast<double>(evaluation.matchedBrightDetections)
        + 0.35 * static_cast<double>(evaluation.matchedBrightProjectedStars);
}

double CameraPlateSolver::SolverContext::narrowGuidedSeedConsistencyScore(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation)
{
    if (!m_useDirectionSeedPreference
        || (!isNarrowField(settings))
        || !evaluation.projectorValid)
    {
        return 0.0;
    }

    double score = 0.0;
    const bool useSeedProjectedBrightGate = usesSeedProjectedBrightGate(settings);
    if (useSeedProjectedBrightGate && (evaluation.seedProjectedMagnitudeSupport > 0.0))
    {
        const double weightedCoverage = std::clamp(
            evaluation.seedProjectedMagnitudeMatchFraction,
            0.0,
            1.0);
        score += 12.0 * weightedCoverage;
        score += 0.8 * std::min(6.0, evaluation.matchedSeedProjectedMagnitudeSupport);
        score -= 4.0 * (1.0 - weightedCoverage);
    }
    else if (useSeedProjectedBrightGate && (evaluation.seedProjectedBrightStars > 0))
    {
        score += 6.0 * std::clamp(evaluation.seedProjectedBrightMatchFraction, 0.0, 1.0);
    }
    if (useSeedProjectedBrightGate) {
        score += 0.35 * static_cast<double>(evaluation.matchedSeedProjectedBrightStars);
    }

    if ((evaluation.prioritySeedRadialChecks >= 4)
        && std::isfinite(evaluation.prioritySeedRadialErrorPixels))
    {
        const double normalizedError = evaluation.prioritySeedRadialErrorPixels
            / std::max(48.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 2.0);
        score += 8.0 / (1.0 + normalizedError * normalizedError);
    }
    if ((evaluation.prioritySeedProjectedChecks >= 1)
        && std::isfinite(evaluation.prioritySeedProjectedErrorPixels))
    {
        const double normalizedError = evaluation.prioritySeedProjectedErrorPixels
            / std::max(48.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 2.0);
        score += 6.0 / (1.0 + 1.5 * normalizedError * normalizedError);
    }
    if (!useSeedProjectedBrightGate && (evaluation.seedRadialMagnitudeSupport > 0.0))
    {
        const double weightedCoverage = std::clamp(
            evaluation.seedRadialMagnitudeMatchFraction,
            0.0,
            1.0);
        score += 16.0 * weightedCoverage;
        score += 0.10 * std::min(80.0, evaluation.matchedSeedRadialMagnitudeSupport);
        score -= 6.0 * (1.0 - weightedCoverage);
    }
    if (!useSeedProjectedBrightGate && (evaluation.seedProjectedMagnitudeSupport > 0.0))
    {
        const double weightedCoverage = std::clamp(
            evaluation.seedProjectedMagnitudeMatchFraction,
            0.0,
            1.0);
        score += 8.0 * weightedCoverage;
        score += 0.12 * std::min(40.0, evaluation.matchedSeedProjectedMagnitudeSupport);
    }

    if ((evaluation.sparseGuidedNamedAnchorMatches >= 2)
        && std::isfinite(evaluation.sparseGuidedAnchorRmsErrorPixels))
    {
        score += 2.0 / (1.0 + std::max(0.0, evaluation.sparseGuidedAnchorRmsErrorPixels) / 8.0);
    }

    return score;
}

QString CameraPlateSolver::SolverContext::finalPassBrightDiagnosticSummary(const FinalMatchPassEvaluation& evaluation)
{
    return QStringLiteral("brightDetections=%1/%2 brightProjected=%3/%4 seedBright=%5/%6 seedMag=%7/%8(%9) seedRadialMag=%10/%11(%12) brightMagErr=%13 brightShape=%14/%15 brightnessErr=%16 magSupport=%17/%18 projectedMag=%19/%20(%21) seedRadial=%22/%23 seedProjected=%24/%25 namedAnchors=%26 namedRms=%27 sparseAnchors=%28 rms=%29 anchorBrightErr=%30")
        .arg(evaluation.matchedBrightDetections)
        .arg(evaluation.brightDetections)
        .arg(evaluation.matchedBrightProjectedStars)
        .arg(evaluation.brightProjectedStars)
        .arg(evaluation.matchedSeedProjectedBrightStars)
        .arg(evaluation.seedProjectedBrightStars)
        .arg(evaluation.matchedSeedProjectedMagnitudeSupport, 0, 'f', 1)
        .arg(evaluation.seedProjectedMagnitudeSupport, 0, 'f', 1)
        .arg(evaluation.seedProjectedMagnitudeMatchFraction, 0, 'f', 3)
        .arg(evaluation.matchedSeedRadialMagnitudeSupport, 0, 'f', 1)
        .arg(evaluation.seedRadialMagnitudeSupport, 0, 'f', 1)
        .arg(evaluation.seedRadialMagnitudeMatchFraction, 0, 'f', 3)
        .arg(evaluation.brightDetectionMagnitudeError, 0, 'f', 2)
        .arg(evaluation.brightCatalogShapeMismatches)
        .arg(evaluation.brightCatalogShapeChecks)
        .arg(evaluation.brightnessRankError, 0, 'f', 3)
        .arg(evaluation.magnitudeWeightedSupport, 0, 'f', 1)
        .arg(evaluation.priorityMagnitudeWeightedSupport, 0, 'f', 1)
        .arg(evaluation.matchedProjectedMagnitudeSupport, 0, 'f', 1)
        .arg(evaluation.projectedMagnitudeSupport, 0, 'f', 1)
        .arg(evaluation.projectedMagnitudeMatchFraction, 0, 'f', 3)
        .arg(evaluation.prioritySeedRadialErrorPixels, 0, 'f', 1)
        .arg(evaluation.prioritySeedRadialChecks)
        .arg(evaluation.prioritySeedProjectedErrorPixels, 0, 'f', 1)
        .arg(evaluation.prioritySeedProjectedChecks)
        .arg(evaluation.namedBrightAnchorMatches)
        .arg(evaluation.namedBrightAnchorRmsErrorPixels, 0, 'f', 2)
        .arg(evaluation.sparseGuidedNamedAnchorMatches)
        .arg(evaluation.sparseGuidedAnchorRmsErrorPixels, 0, 'f', 2)
        .arg(evaluation.sparseGuidedAnchorBrightnessRankError, 0, 'f', 3);
}

bool CameraPlateSolver::SolverContext::hasHighConfidenceSparseGuidedAnchors(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation)
{
    if (!evaluation.projectorValid
        || !evaluation.pose.sparseGuidedPair
        || !m_useDirectionSeedPreference
        || (!isNarrowField(settings))
        || (evaluation.sparseGuidedNamedAnchorMatches < 2)
        || !std::isfinite(evaluation.sparseGuidedAnchorRmsErrorPixels)
        || !std::isfinite(evaluation.sparseGuidedAnchorBrightnessRankError))
    {
        return false;
    }

    const double maxAnchorRms = std::min(
        static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.45,
        10.0);
    const double directionDistance = directionSeedAngularDistanceDegrees(evaluation.pose);
    const double maxAnchorDirectionDistance = std::max(1.6, settings.m_fov * 1.25);
    const bool enoughBrightDetectionsToCrossCheck =
        isLowMagnitudeNarrowGuidedSolve(settings)
        && (evaluation.brightDetections >= 12);
    if (enoughBrightDetectionsToCrossCheck
        && (evaluation.matchedBrightDetections < 3))
    {
        return false;
    }
    return (evaluation.sparseGuidedAnchorRmsErrorPixels <= maxAnchorRms)
        && (evaluation.sparseGuidedAnchorBrightnessRankError <= 0.86)
        && (directionDistance <= maxAnchorDirectionDistance);
}

bool CameraPlateSolver::SolverContext::hasHighConfidenceGuidedTriangleSupport(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation)
{
    if (!evaluation.projectorValid
        || !evaluation.pose.guidedTriangle
        || !m_useDirectionSeedPreference
        || (!isNarrowField(settings))
        || (evaluation.finalMatches.size() < std::max(settings.m_plateSolveMinMatches + 12, 16))
        || !std::isfinite(evaluation.rmsErrorPixels)
        || (evaluation.rmsErrorPixels > maxDirectionSeedRmsError(settings, evaluation.finalMatches.size()))
        || ((evaluation.brightCatalogShapeChecks >= 1) && (evaluation.brightCatalogShapeMismatches > 0)))
    {
        return false;
    }

    const double directionDistance = directionSeedAngularDistanceDegrees(evaluation.pose);
    const double maxDirectionDistance = std::max(1.6, settings.m_fov * 1.25);
    const double fovDelta = std::fabs(evaluation.pose.fovDegrees - settings.m_fov);
    const double maxFovDelta = std::max(0.08, settings.m_fov * 0.08);
    if ((directionDistance > maxDirectionDistance) || (fovDelta > maxFovDelta)) {
        return false;
    }

    const bool projectedBrightSupport =
        (evaluation.matchedBrightProjectedStars >= 3)
        && ((evaluation.brightProjectedStars < 10)
            || (evaluation.matchedBrightProjectedStars >= 4))
        && (evaluation.brightProjectedMatchFraction >= 0.30);
    const bool detectedBrightSupport =
        (evaluation.brightDetections < 12)
        || ((evaluation.matchedBrightDetections >= 3)
            && (evaluation.brightDetectionMagnitudeError <= 1.60));
    const bool projectedMagnitudeSupport =
        (evaluation.projectedMagnitudeSupport <= 0.0)
        || ((evaluation.matchedProjectedMagnitudeSupport >= 16.0)
            && (evaluation.projectedMagnitudeMatchFraction >= 0.55));
    const bool seedMagnitudeSupport =
        (evaluation.seedProjectedMagnitudeSupport <= 0.0)
        || ((evaluation.matchedSeedProjectedMagnitudeSupport >= 10.0)
            && (evaluation.seedProjectedMagnitudeMatchFraction >= 0.35));
    const bool brightnessOrderReasonable =
        !std::isfinite(evaluation.brightnessRankError)
        || (evaluation.brightnessRankError <= 0.70);

    return projectedBrightSupport
        && detectedBrightSupport
        && projectedMagnitudeSupport
        && seedMagnitudeSupport
        && brightnessOrderReasonable;
}

double CameraPlateSolver::SolverContext::sparseGuidedAnchorRankingScore(const FinalMatchPassEvaluation& evaluation)
{
    if (!std::isfinite(evaluation.sparseGuidedAnchorRmsErrorPixels)
        || !std::isfinite(evaluation.sparseGuidedAnchorBrightnessRankError))
    {
        return std::numeric_limits<double>::infinity();
    }

    const double meanMagnitude = std::isfinite(evaluation.sparseGuidedAnchorMeanMagnitude)
        ? evaluation.sparseGuidedAnchorMeanMagnitude
        : 12.0;
    const double directionDistance = directionSeedAngularDistanceDegrees(evaluation.pose);
    return evaluation.sparseGuidedAnchorRmsErrorPixels
        + 8.0 * evaluation.sparseGuidedAnchorBrightnessRankError
        + 0.12 * meanMagnitude
        + 3.5 * directionDistance;
}

double CameraPlateSolver::SolverContext::namedBrightAnchorEvidenceScore(const FinalMatchPassEvaluation& evaluation)
{
    if ((evaluation.namedBrightAnchorMatches <= 0)
        || !std::isfinite(evaluation.namedBrightAnchorRmsErrorPixels))
    {
        return 0.0;
    }

    const double meanMagnitude = std::isfinite(evaluation.namedBrightAnchorMeanMagnitude)
        ? evaluation.namedBrightAnchorMeanMagnitude
        : 12.0;
    const double rmsQuality = 1.0 / (1.0 + std::max(0.0, evaluation.namedBrightAnchorRmsErrorPixels) / 8.0);
    const double magnitudeQuality = 1.0 / (1.0 + std::max(0.0, meanMagnitude - 7.0) / 5.0);
    return static_cast<double>(evaluation.namedBrightAnchorMatches) * (1.0 + rmsQuality + 0.6 * magnitudeQuality);
}

double CameraPlateSolver::SolverContext::finalMatchPassScore(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation)
{
    if (!evaluation.projectorValid) {
        return -std::numeric_limits<double>::infinity();
    }

    Evaluation pose = evaluation.pose;
    pose.matchCount = evaluation.finalMatches.size();
    pose.matches = evaluation.finalMatches;
    pose.rmsErrorPixels = evaluation.rmsErrorPixels;
    pose.brightnessRankError = evaluation.brightnessRankError;
    pose.meanCatalogMagnitude = evaluation.meanCatalogMagnitude;

    const double normalizationRadius = std::min(
        std::max(1.0, m_weakModeNormalizationPixels),
        std::max(1.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius)));
    double matchSupport = wideFinalPassMatchWeight(settings, evaluation.finalMatches.size());
    if (m_useDirectionSeedPreference
        && (isNarrowField(settings))
        && (evaluation.magnitudeWeightedSupport > 0.0))
    {
        const double prioritySupport = narrowGuidedMagnitudePriorityScore(evaluation);
        const double usefulSupportCap = static_cast<double>(
            std::max(settings.m_plateSolveMinMatches + 14, 18));
        matchSupport = std::min(prioritySupport, usefulSupportCap)
            + 0.15 * std::log1p(std::max(0.0, prioritySupport - usefulSupportCap));
    }

    return matchSupport
        * evaluationRmsQuality(pose, normalizationRadius)
        * brightnessAffinity(pose)
        * wideFinalPassMagnitudeAffinity(evaluation)
        * brightDetectionCoverageAffinity(settings, evaluation)
        * brightProjectedCoverageAffinity(settings, evaluation)
        * brightDetectionMagnitudeAffinity(settings, evaluation)
        * projectedMagnitudeCoverageAffinity(settings, evaluation)
        * prioritySeedProjectedAffinity(evaluation, settings.m_plateSolveFinalMatchRadius)
        * prioritySeedRadialAffinity(evaluation, settings.m_plateSolveFinalMatchRadius)
        * seedProjectedMagnitudeCoverageAffinity(evaluation)
        * seedRadialMagnitudeCoverageAffinity(evaluation)
        * (m_useDirectionSeedPreference && (isNarrowField(settings))
            ? (1.0 + 0.18 * namedBrightAnchorEvidenceScore(evaluation))
            : 1.0)
        * directionSeedAffinity(pose)
        * fovSeedAffinity(pose)
        * allSkyZenithAffinity(pose);
}

double CameraPlateSolver::SolverContext::narrowGuidedEvidenceScore(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation)
{
    if (!m_useDirectionSeedPreference
        || (!isNarrowField(settings))
        || !evaluation.projectorValid
        || !std::isfinite(evaluation.rmsErrorPixels))
    {
        return -std::numeric_limits<double>::infinity();
    }

    const int usefulMatchCap = std::max(settings.m_plateSolveMinMatches + 14, 18);
    const int matchCount = static_cast<int>(evaluation.finalMatches.size());
    const double matchSupport = std::min(matchCount, usefulMatchCap)
        + 0.05 * std::log1p(static_cast<double>(std::max(0, matchCount - usefulMatchCap)));
    const double rmsPenalty = 6.0 * evaluation.rmsErrorPixels
        / std::max(1.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius));
    const double directionPenalty = 1.8 * directionSeedAngularDistanceDegrees(evaluation.pose)
        / std::max(0.25, static_cast<double>(settings.m_fov));
    const double fovPenalty = 2.0
        * std::fabs(evaluation.pose.fovDegrees - m_directionSeedReferenceFovDegrees)
        / std::max(0.03, static_cast<double>(settings.m_fov) * 0.03);

    return 1.20 * narrowGuidedMagnitudePriorityScore(evaluation)
        + 2.75 * narrowGuidedSeedConsistencyScore(settings, evaluation)
        + 2.25 * narrowGuidedBrightConsistencyScore(settings, evaluation)
        + 0.55 * matchSupport
        - rmsPenalty
        - directionPenalty
        - fovPenalty;
}

double CameraPlateSolver::SolverContext::finalMatchPassEvidenceScore(const CameraSettings& settings, const FinalMatchPassEvaluation& evaluation)
{
    if (!evaluation.projectorValid
        || !std::isfinite(evaluation.rmsErrorPixels))
    {
        return -std::numeric_limits<double>::infinity();
    }

    const bool narrowGuided =
        m_useDirectionSeedPreference && (isNarrowField(settings));
    const int usefulMatchCap = narrowGuided
        ? std::max(settings.m_plateSolveMinMatches + 14, 18)
        : std::max(settings.m_plateSolveMinMatches + 8, 12);
    const int matchCount = static_cast<int>(evaluation.finalMatches.size());
    double matchEvidence = std::min(matchCount, usefulMatchCap)
        + 0.20 * std::log1p(static_cast<double>(std::max(0, matchCount - usefulMatchCap)));
    if (narrowGuided && (evaluation.magnitudeWeightedSupport > 0.0))
    {
        const double prioritySupport = narrowGuidedMagnitudePriorityScore(evaluation);
        matchEvidence = std::max(
            matchEvidence,
            std::min(prioritySupport, static_cast<double>(usefulMatchCap))
                + 0.10 * std::log1p(std::max(0.0, prioritySupport - static_cast<double>(usefulMatchCap))));
    }

    const double radius = std::max(
        1.0,
        static_cast<double>(settings.m_plateSolveFinalMatchRadius));
    const double rmsPenalty = 5.5 * std::min(4.0, evaluation.rmsErrorPixels / radius);
    const double maxPenalty = std::isfinite(evaluation.maxErrorPixels)
        ? 0.8 * std::min(4.0, evaluation.maxErrorPixels / std::max(radius, 1.0))
        : 0.0;
    const double brightnessPenalty = std::isfinite(evaluation.brightnessRankError)
        ? 5.0 * std::clamp(evaluation.brightnessRankError, 0.0, 1.5)
        : 0.0;
    const double outlierPenalty = 0.55 * static_cast<double>(evaluation.outlierCount);

    double score = 1.25 * matchEvidence
        - rmsPenalty
        - maxPenalty
        - brightnessPenalty
        - outlierPenalty;

    if (std::isfinite(evaluation.meanCatalogMagnitude))
    {
        const double referenceMagnitude = narrowGuided ? 12.0 : 5.0;
        score -= 0.18 * std::max(0.0, evaluation.meanCatalogMagnitude - referenceMagnitude);
    }

    if (evaluation.brightDetections >= 3)
    {
        const double coverage = std::clamp(evaluation.brightDetectionMatchFraction, 0.0, 1.0);
        score += (narrowGuided ? 7.0 : 5.0) * coverage;
        score -= (narrowGuided ? 4.0 : 2.5) * (1.0 - coverage);
    }
    if (evaluation.brightProjectedStars >= (narrowGuided ? 2 : 3))
    {
        const double coverage = std::clamp(evaluation.brightProjectedMatchFraction, 0.0, 1.0);
        score += (narrowGuided ? 8.0 : 5.5) * coverage;
        score -= (narrowGuided ? 5.0 : 3.0) * (1.0 - coverage);
    }
    if (evaluation.projectedMagnitudeSupport > 0.0)
    {
        const double coverage = std::clamp(evaluation.projectedMagnitudeMatchFraction, 0.0, 1.0);
        score += (narrowGuided ? 8.0 : 4.0) * coverage;
        score += 0.08 * std::min(80.0, evaluation.matchedProjectedMagnitudeSupport);
    }
    if (evaluation.seedProjectedMagnitudeSupport > 0.0)
    {
        const double coverage = std::clamp(evaluation.seedProjectedMagnitudeMatchFraction, 0.0, 1.0);
        score += (narrowGuided ? 6.0 : 2.0) * coverage;
        score += 0.05 * std::min(80.0, evaluation.matchedSeedProjectedMagnitudeSupport);
    }
    if ((evaluation.prioritySeedProjectedChecks >= 1)
        && std::isfinite(evaluation.prioritySeedProjectedErrorPixels))
    {
        const double seedProjectedAffinity = prioritySeedProjectedAffinity(
            evaluation,
            static_cast<double>(settings.m_plateSolveFinalMatchRadius));
        score += (narrowGuided ? 5.0 : 1.5) * seedProjectedAffinity;
        score -= (narrowGuided ? 3.0 : 1.0) * (1.0 - seedProjectedAffinity);
    }
    if (evaluation.seedRadialMagnitudeSupport > 0.0)
    {
        const double coverage = std::clamp(evaluation.seedRadialMagnitudeMatchFraction, 0.0, 1.0);
        score += (narrowGuided ? 7.0 : 2.0) * coverage;
        score += 0.05 * std::min(80.0, evaluation.matchedSeedRadialMagnitudeSupport);
    }
    if (evaluation.brightCatalogShapeChecks > 0)
    {
        const double mismatchFraction = std::clamp(
            static_cast<double>(evaluation.brightCatalogShapeMismatches)
                / static_cast<double>(evaluation.brightCatalogShapeChecks),
            0.0,
            1.0);
        score -= 4.0 * mismatchFraction;
    }
    if (evaluation.brightDetections >= 3)
    {
        const double magnitudeError = std::max(0.0, evaluation.brightDetectionMagnitudeError);
        score -= (narrowGuided ? 4.0 : 2.5) * std::min(2.0, magnitudeError * magnitudeError);
    }

    if (m_useDirectionSeedPreference)
    {
        const double directionDistance = directionSeedAngularDistanceDegrees(evaluation.pose);
        const double directionScale = narrowGuided
            ? std::max(0.25, static_cast<double>(settings.m_fov))
            : std::max(5.0, static_cast<double>(settings.m_plateSolveAzElSearchRadius));
        score -= 2.0 * std::min(9.0, std::pow(directionDistance / directionScale, 2.0));

        const double fovDelta = std::fabs(evaluation.pose.fovDegrees - m_directionSeedReferenceFovDegrees);
        const double fovScale = narrowGuided
            ? std::max(0.03, static_cast<double>(settings.m_fov) * 0.03)
            : std::max(1.0, static_cast<double>(settings.m_fov) * 0.08);
        score -= 1.5 * std::min(9.0, std::pow(fovDelta / fovScale, 2.0));

        if (m_directionSeedHasRollPreference)
        {
            const double rollDelta = angularDistanceDegrees(
                evaluation.pose.rollDegrees,
                m_directionSeedReferenceRollDegrees);
            score -= 1.0 * std::min(9.0, std::pow(
                rollDelta / std::max(1.0, m_directionSeedRollScaleDegrees),
                2.0));
        }
    }

    if (narrowGuided) {
        score += 0.70 * narrowGuidedSeedConsistencyScore(settings, evaluation);
        score += 0.45 * narrowGuidedBrightConsistencyScore(settings, evaluation);
        score += 2.25 * namedBrightAnchorEvidenceScore(evaluation);
    }

    return score;
}

bool CameraPlateSolver::SolverContext::isBetterWeakModeFinalMatchPass(const CameraSettings& settings, const QVector<CameraPipelineStarDetection>& starDetections, bool blindMode, const FinalMatchPassEvaluation& candidate, const FinalMatchPassEvaluation& best)
{
    if (!candidate.projectorValid) {
        return false;
    }
    if (!best.projectorValid) {
        return true;
    }

    const bool candidateMeetsMinMatches = candidate.finalMatches.size() >= settings.m_plateSolveMinMatches;
    const bool bestMeetsMinMatches = best.finalMatches.size() >= settings.m_plateSolveMinMatches;
    const bool candidateBlindAccepted = blindMode
        && candidateMeetsMinMatches
        && isAcceptableBlindSolve(settings, starDetections, candidate.finalMatches, candidate.rmsErrorPixels, candidate.maxErrorPixels);
    const bool bestBlindAccepted = blindMode
        && bestMeetsMinMatches
        && isAcceptableBlindSolve(settings, starDetections, best.finalMatches, best.rmsErrorPixels, best.maxErrorPixels);

    if (candidateMeetsMinMatches != bestMeetsMinMatches) {
        return candidateMeetsMinMatches;
    }

    if (isWidePlateSolveContext(settings))
    {
        // NB: a "tight named anchor" early preference (prefer the candidate whose named bright
        // anchor matched within ~30% of the match radius, ahead of the broad bright-anchor gate)
        // was tried here and removed. It was introduced disabled (multiplier * 0.00) in
        // "Fix failing test" and re-enabling it at * 0.30 regresses 7 wide REAL cases
        // (stars-wide-2/3/4/5/7/8): a single tight named-anchor RMS is not by itself reliable
        // evidence of the correct wide pose, so it prefers contaminated candidates. The broad
        // bright-anchor support gate below is the wide-field selector.
        const bool candidateBrightAnchorAccepted =
            hasAcceptableWideBrightAnchorSupport(settings, starDetections, candidate);
        const bool bestBrightAnchorAccepted =
            hasAcceptableWideBrightAnchorSupport(settings, starDetections, best);
        if (qEnvironmentVariableIsSet("SDRANGEL_CAMERA_SOLVER_DEBUG_COMPARE")) {
            qCDebug(cameraPlateSolverLog).noquote().nospace()
                << "COMPARE"
                << " cAz=" << candidate.pose.azimuthDegrees
                << " cM=" << candidate.finalMatches.size()
                << " cRms=" << candidate.rmsErrorPixels
                << " cBD=" << candidate.brightDetections
                << " cMBD=" << candidate.matchedBrightDetections
                << " cBME=" << candidate.brightDetectionMagnitudeError
                << " cAnch=" << candidate.namedBrightAnchorMatches
                << " cAnchRms=" << candidate.namedBrightAnchorRmsErrorPixels
                << " cBrightOk=" << candidateBrightAnchorAccepted
                << " | bAz=" << best.pose.azimuthDegrees
                << " bM=" << best.finalMatches.size()
                << " bRms=" << best.rmsErrorPixels
                << " bBD=" << best.brightDetections
                << " bMBD=" << best.matchedBrightDetections
                << " bBME=" << best.brightDetectionMagnitudeError
                << " bAnch=" << best.namedBrightAnchorMatches
                << " bAnchRms=" << best.namedBrightAnchorRmsErrorPixels
                << " bBrightOk=" << bestBrightAnchorAccepted;
        }
        if (candidateBrightAnchorAccepted != bestBrightAnchorAccepted) {
            return candidateBrightAnchorAccepted;
        }
    }

    if (blindMode && (candidateBlindAccepted != bestBlindAccepted)) {
        return candidateBlindAccepted;
    }

    const bool useNarrowGuidedMatchCap = m_useDirectionSeedPreference
        && (isNarrowField(settings));
    if (useNarrowGuidedMatchCap)
    {
        const int candidateMatches = static_cast<int>(candidate.finalMatches.size());
        const int bestMatches = static_cast<int>(best.finalMatches.size());
        const double namedAnchorRmsCap = std::min(
            maxDirectionSeedRmsError(settings, std::max(candidateMatches, bestMatches)) * 1.10,
            static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.75);
        const bool candidateNamedAnchorAccepted =
            (candidate.namedBrightAnchorMatches >= 2)
            && std::isfinite(candidate.namedBrightAnchorRmsErrorPixels)
            && (candidate.namedBrightAnchorRmsErrorPixels <= namedAnchorRmsCap)
            && (candidate.rmsErrorPixels <= std::min(
                maxDirectionSeedRmsError(settings, candidateMatches) * 1.15,
                static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.78));
        const bool bestNamedAnchorAccepted =
            (best.namedBrightAnchorMatches >= 2)
            && std::isfinite(best.namedBrightAnchorRmsErrorPixels)
            && (best.namedBrightAnchorRmsErrorPixels <= namedAnchorRmsCap)
            && (best.rmsErrorPixels <= std::min(
                maxDirectionSeedRmsError(settings, bestMatches) * 1.15,
                static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.78));
        const double strongNamedAnchorRmsCap = std::max(
            namedAnchorRmsCap,
            static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.65);
        const bool candidateHasStrongNamedAnchorSet =
            (candidate.namedBrightAnchorMatches >= 3)
            && std::isfinite(candidate.namedBrightAnchorRmsErrorPixels)
            && (candidate.namedBrightAnchorRmsErrorPixels <= strongNamedAnchorRmsCap)
            && (candidateMatches >= std::max(settings.m_plateSolveMinMatches + 6, 10));
        const bool bestHasStrongNamedAnchorSet =
            (best.namedBrightAnchorMatches >= 3)
            && std::isfinite(best.namedBrightAnchorRmsErrorPixels)
            && (best.namedBrightAnchorRmsErrorPixels <= strongNamedAnchorRmsCap)
            && (bestMatches >= std::max(settings.m_plateSolveMinMatches + 6, 10));
        if (candidateHasStrongNamedAnchorSet != bestHasStrongNamedAnchorSet) {
            const bool candidateHasStrongDenseGuidedSupport =
                cachedHasStrongDenseNarrowGuidedFinalPass(settings, candidate);
            const bool bestHasStrongDenseGuidedSupport =
                cachedHasStrongDenseNarrowGuidedFinalPass(settings, best);
            if (candidateHasStrongDenseGuidedSupport != bestHasStrongDenseGuidedSupport) {
                return candidateHasStrongDenseGuidedSupport;
            }
            return candidateHasStrongNamedAnchorSet;
        }
        const bool candidateHasStrongDenseGuidedSupport =
            cachedHasStrongDenseNarrowGuidedFinalPass(settings, candidate);
        const bool bestHasStrongDenseGuidedSupport =
            cachedHasStrongDenseNarrowGuidedFinalPass(settings, best);
        if (candidateHasStrongDenseGuidedSupport != bestHasStrongDenseGuidedSupport) {
            return candidateHasStrongDenseGuidedSupport;
        }
        if (candidateNamedAnchorAccepted != bestNamedAnchorAccepted)
        {
            const bool candidateHasEnoughSupport =
                candidateMatches >= std::max(settings.m_plateSolveMinMatches + 10, 16);
            const bool bestHasEnoughSupport =
                bestMatches >= std::max(settings.m_plateSolveMinMatches + 10, 16);
            if (candidateHasEnoughSupport && bestHasEnoughSupport) {
                return candidateNamedAnchorAccepted;
            }
        }
        if (candidateNamedAnchorAccepted && bestNamedAnchorAccepted)
        {
            const double namedAnchorDelta =
                namedBrightAnchorEvidenceScore(candidate) - namedBrightAnchorEvidenceScore(best);
            if (std::fabs(namedAnchorDelta) >= 1.25) {
                return namedAnchorDelta > 0.0;
            }
        }

        const int seedConsistencyMatchTolerance = std::max(
            20,
            static_cast<int>(std::ceil(static_cast<double>(std::max(candidateMatches, bestMatches)) * 0.25)));
        const bool seedConsistencyComparableSupport =
            (candidateMatches >= std::max(settings.m_plateSolveMinMatches + 4, bestMatches - seedConsistencyMatchTolerance))
            && (bestMatches >= std::max(settings.m_plateSolveMinMatches + 4, candidateMatches - seedConsistencyMatchTolerance));
        const double seedConsistencyRmsTolerance = std::max(
            3.0,
            static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.20);
        const bool seedConsistencyComparableRms =
            (candidate.rmsErrorPixels <= (best.rmsErrorPixels + seedConsistencyRmsTolerance))
            && (best.rmsErrorPixels <= (candidate.rmsErrorPixels + seedConsistencyRmsTolerance));
        if (!plateSolveStartUsesRoll(settings)
            && seedConsistencyComparableSupport
            && seedConsistencyComparableRms)
        {
            const double candidateSeedConsistency = cachedNarrowGuidedSeedConsistencyScore(settings, candidate);
            const double bestSeedConsistency = cachedNarrowGuidedSeedConsistencyScore(settings, best);
            const double seedConsistencyDelta = candidateSeedConsistency - bestSeedConsistency;
            if (std::fabs(seedConsistencyDelta) >= 1.5) {
                return seedConsistencyDelta > 0.0;
            }
        }

        if (seedConsistencyComparableSupport
            && seedConsistencyComparableRms
            && ((candidate.seedProjectedMagnitudeSupport > 0.0)
                || (best.seedProjectedMagnitudeSupport > 0.0)))
        {
            const double candidateSeedProjectedSupport =
                candidate.matchedSeedProjectedMagnitudeSupport;
            const double bestSeedProjectedSupport =
                best.matchedSeedProjectedMagnitudeSupport;
            const double seedProjectedSupportDelta =
                candidateSeedProjectedSupport - bestSeedProjectedSupport;
            const double meaningfulSeedProjectedSupportDelta = std::max(
                4.0,
                0.10 * std::max(candidateSeedProjectedSupport, bestSeedProjectedSupport));
            if (std::fabs(seedProjectedSupportDelta) >= meaningfulSeedProjectedSupportDelta) {
                return seedProjectedSupportDelta > 0.0;
            }

            const double seedProjectedFractionDelta =
                candidate.seedProjectedMagnitudeMatchFraction - best.seedProjectedMagnitudeMatchFraction;
            if (std::fabs(seedProjectedFractionDelta) >= 0.04) {
                return seedProjectedFractionDelta > 0.0;
            }
        }
        if (seedConsistencyComparableSupport
            && seedConsistencyComparableRms
            && (candidate.prioritySeedProjectedChecks >= 1)
            && (best.prioritySeedProjectedChecks >= 1)
            && std::isfinite(candidate.prioritySeedProjectedErrorPixels)
            && std::isfinite(best.prioritySeedProjectedErrorPixels))
        {
            const double projectedErrorDelta =
                best.prioritySeedProjectedErrorPixels - candidate.prioritySeedProjectedErrorPixels;
            const double meaningfulProjectedErrorDelta = std::max(
                10.0,
                static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.75);
            if (std::fabs(projectedErrorDelta) >= meaningfulProjectedErrorDelta) {
                return projectedErrorDelta > 0.0;
            }
        }

        const bool candidateHasStrongSparseAnchors =
            hasHighConfidenceSparseGuidedAnchors(settings, candidate);
        const bool bestHasStrongSparseAnchors =
            hasHighConfidenceSparseGuidedAnchors(settings, best);
        const double candidateAnchorMagnitudeSupport = narrowGuidedMagnitudePriorityScore(candidate);
        const double bestAnchorMagnitudeSupport = narrowGuidedMagnitudePriorityScore(best);
        const double anchorMagnitudeSupportTolerance = std::max(
            4.0,
            0.08 * std::max(candidateAnchorMagnitudeSupport, bestAnchorMagnitudeSupport));
        const double anchorRmsTolerance = std::max(2.5, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.15);
        const bool anchorCandidatesHaveComparableRms =
            (candidate.rmsErrorPixels <= (best.rmsErrorPixels + anchorRmsTolerance))
            && (best.rmsErrorPixels <= (candidate.rmsErrorPixels + anchorRmsTolerance));
        if (candidateHasStrongSparseAnchors != bestHasStrongSparseAnchors)
        {
            if (anchorCandidatesHaveComparableRms
                && (std::fabs(candidateAnchorMagnitudeSupport - bestAnchorMagnitudeSupport) >= anchorMagnitudeSupportTolerance))
            {
                return candidateAnchorMagnitudeSupport > bestAnchorMagnitudeSupport;
            }
            return candidateHasStrongSparseAnchors;
        }
        if (candidateHasStrongSparseAnchors && bestHasStrongSparseAnchors)
        {
            const double candidateAnchorScore = sparseGuidedAnchorRankingScore(candidate);
            const double bestAnchorScore = sparseGuidedAnchorRankingScore(best);
            const double anchorScoreDelta = candidateAnchorScore - bestAnchorScore;
            const int candidateMatches = static_cast<int>(candidate.finalMatches.size());
            const int bestMatches = static_cast<int>(best.finalMatches.size());
            const int matchDelta = candidateMatches - bestMatches;
            const int meaningfulMatchDelta = std::max(
                10,
                static_cast<int>(std::ceil(static_cast<double>(std::max(candidateMatches, bestMatches)) * 0.10)));
            const double rmsTolerance = std::max(2.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.10);
            if ((std::abs(matchDelta) >= meaningfulMatchDelta)
                && (candidate.rmsErrorPixels <= (best.rmsErrorPixels + rmsTolerance))
                && (best.rmsErrorPixels <= (candidate.rmsErrorPixels + rmsTolerance)))
            {
                return matchDelta > 0;
            }

            const int seedRadialMatchTolerance = std::max(
                meaningfulMatchDelta,
                static_cast<int>(std::ceil(static_cast<double>(std::max(candidateMatches, bestMatches)) * 0.25)));
            if (anchorCandidatesHaveComparableRms
                && shouldPreferSeedRadialConsistency(candidate, best, seedRadialMatchTolerance))
            {
                return true;
            }
            if (anchorCandidatesHaveComparableRms
                && shouldPreferSeedRadialConsistency(best, candidate, seedRadialMatchTolerance))
            {
                return false;
            }

            const bool candidateHasComparableSupport =
                candidateMatches >= std::max(settings.m_plateSolveMinMatches + 4, static_cast<int>(std::floor(bestMatches * 0.90)));
            const bool bestHasComparableSupport =
                bestMatches >= std::max(settings.m_plateSolveMinMatches + 4, static_cast<int>(std::floor(candidateMatches * 0.90)));
            if ((std::fabs(anchorScoreDelta) >= 1.0)
                && candidateHasComparableSupport
                && bestHasComparableSupport)
            {
                return anchorScoreDelta < 0.0;
            }

            if (anchorCandidatesHaveComparableRms
                && (std::fabs(candidateAnchorMagnitudeSupport - bestAnchorMagnitudeSupport) >= anchorMagnitudeSupportTolerance))
            {
                return candidateAnchorMagnitudeSupport > bestAnchorMagnitudeSupport;
            }

            const int actualAnchorMatchDelta = candidateMatches - bestMatches;
            if ((std::abs(actualAnchorMatchDelta) >= 6)
                && (std::fabs(anchorScoreDelta) <= 2.0)
                && (candidate.rmsErrorPixels <= (best.rmsErrorPixels + rmsTolerance))
                && (best.rmsErrorPixels <= (candidate.rmsErrorPixels + rmsTolerance)))
            {
                return actualAnchorMatchDelta > 0;
            }
        }
    }

    if (m_useDirectionSeedPreference)
    {
        const bool candidateDirectionAccepted = candidateMeetsMinMatches
            && (isAcceptableDirectionSeedSolve(
                    settings,
                    starDetections,
                    candidate.finalMatches,
                    candidate.rmsErrorPixels,
                    candidate.maxErrorPixels)
                || isAcceptableSparseGuidedRankingFinalPass(settings, candidate))
            && hasAcceptableGuidedFinalBrightnessConsistency(settings, candidate);
        const bool bestDirectionAccepted = bestMeetsMinMatches
            && (isAcceptableDirectionSeedSolve(
                    settings,
                    starDetections,
                    best.finalMatches,
                    best.rmsErrorPixels,
                    best.maxErrorPixels)
                || isAcceptableSparseGuidedRankingFinalPass(settings, best))
            && hasAcceptableGuidedFinalBrightnessConsistency(settings, best);
        if (candidateDirectionAccepted != bestDirectionAccepted) {
            return candidateDirectionAccepted;
        }

        const bool candidateBrightnessAccepted = hasAcceptableGuidedFinalBrightnessConsistency(settings, candidate);
        const bool bestBrightnessAccepted = hasAcceptableGuidedFinalBrightnessConsistency(settings, best);
        if (candidateBrightnessAccepted != bestBrightnessAccepted) {
            return candidateBrightnessAccepted;
        }

        if (useNarrowGuidedMatchCap
            && candidateDirectionAccepted
            && bestDirectionAccepted
            && candidateBrightnessAccepted
            && bestBrightnessAccepted)
        {
            const double candidateEvidenceScore = std::max(
                narrowGuidedEvidenceScore(settings, candidate),
                finalMatchPassEvidenceScore(settings, candidate));
            const double bestEvidenceScore = std::max(
                narrowGuidedEvidenceScore(settings, best),
                finalMatchPassEvidenceScore(settings, best));
            const double evidenceScale = std::max({
                1.0,
                std::fabs(candidateEvidenceScore),
                std::fabs(bestEvidenceScore)
            });
            if (std::fabs(candidateEvidenceScore - bestEvidenceScore)
                >= std::max(2.0, evidenceScale * 0.08))
            {
                return candidateEvidenceScore > bestEvidenceScore;
            }
        }

        if (isLowMagnitudeNarrowGuidedSolve(settings)
            && candidateDirectionAccepted
            && bestDirectionAccepted
            && candidateBrightnessAccepted
            && bestBrightnessAccepted)
        {
            const int actualMatchDelta =
                static_cast<int>(candidate.finalMatches.size()) - static_cast<int>(best.finalMatches.size());
            const double rmsTolerance = std::max(2.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.10);
            if ((std::abs(actualMatchDelta) >= 16)
                && (candidate.rmsErrorPixels <= (best.rmsErrorPixels + rmsTolerance))
                && (best.rmsErrorPixels <= (candidate.rmsErrorPixels + rmsTolerance)))
            {
                const double candidateBrightScore = cachedNarrowGuidedBrightConsistencyScore(settings, candidate);
                const double bestBrightScore = cachedNarrowGuidedBrightConsistencyScore(settings, best);
                if (std::fabs(candidateBrightScore - bestBrightScore) <= 1.0) {
                    return actualMatchDelta > 0;
                }
            }
        }
    }

    const int narrowGuidedMatchCap = std::max(settings.m_plateSolveMinMatches + 14, 18);
    const int candidateComparableMatchCount = useNarrowGuidedMatchCap
        ? std::min(static_cast<int>(candidate.finalMatches.size()), narrowGuidedMatchCap)
        : static_cast<int>(candidate.finalMatches.size());
    const int bestComparableMatchCount = useNarrowGuidedMatchCap
        ? std::min(static_cast<int>(best.finalMatches.size()), narrowGuidedMatchCap)
        : static_cast<int>(best.finalMatches.size());
    const int finalMatchDelta = candidateComparableMatchCount - bestComparableMatchCount;
    const int actualFinalMatchDelta = static_cast<int>(candidate.finalMatches.size()) - static_cast<int>(best.finalMatches.size());
    const int narrowGuidedTieBreakMatchTolerance = std::max(3, settings.m_plateSolveMinMatches + 1);
    const bool narrowGuidedActualMatchCountsAreClose =
        std::abs(actualFinalMatchDelta) <= narrowGuidedTieBreakMatchTolerance;

    if (useNarrowGuidedMatchCap && (std::abs(finalMatchDelta) <= 1))
    {
        const bool useSeedProjectedBrightTieBreak = usesSeedProjectedBrightGate(settings);
        const int seedBrightDelta =
            candidate.matchedSeedProjectedBrightStars - best.matchedSeedProjectedBrightStars;
        if (useSeedProjectedBrightTieBreak
            && ((candidate.seedProjectedBrightStars > 0) || (best.seedProjectedBrightStars > 0))
            && (seedBrightDelta != 0))
        {
            return seedBrightDelta > 0;
        }

        const double seedBrightFractionDelta =
            candidate.seedProjectedBrightMatchFraction - best.seedProjectedBrightMatchFraction;
        if (useSeedProjectedBrightTieBreak
            && ((candidate.seedProjectedBrightStars >= 2) || (best.seedProjectedBrightStars >= 2))
            && (std::fabs(seedBrightFractionDelta) >= 0.25))
        {
            return seedBrightFractionDelta > 0.0;
        }
    }

    if (m_useDirectionSeedPreference && (isNarrowField(settings)))
    {
        const double candidateFovDelta = std::fabs(candidate.pose.fovDegrees - m_directionSeedReferenceFovDegrees);
        const double bestFovDelta = std::fabs(best.pose.fovDegrees - m_directionSeedReferenceFovDegrees);
        const double fovDelta = candidateFovDelta - bestFovDelta;
        const double meaningfulFovDelta = std::max(0.02, static_cast<double>(settings.m_fov) * 0.02);
        const double rmsTolerance = std::max(3.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.20);
        if (narrowGuidedActualMatchCountsAreClose
            && (std::fabs(fovDelta) >= meaningfulFovDelta)
            && (candidate.rmsErrorPixels <= (best.rmsErrorPixels + rmsTolerance))
            && (best.rmsErrorPixels <= (candidate.rmsErrorPixels + rmsTolerance)))
        {
            return fovDelta < 0.0;
        }

        const double candidateDirectionDelta = directionSeedAngularDistanceDegrees(candidate.pose);
        const double bestDirectionDelta = directionSeedAngularDistanceDegrees(best.pose);
        const double directionDelta = candidateDirectionDelta - bestDirectionDelta;
        const double meaningfulDirectionDelta = std::max(0.18, static_cast<double>(settings.m_fov) * 0.15);
        if (narrowGuidedActualMatchCountsAreClose
            && (std::fabs(directionDelta) >= meaningfulDirectionDelta)
            && (candidate.rmsErrorPixels <= (best.rmsErrorPixels + rmsTolerance))
            && (best.rmsErrorPixels <= (candidate.rmsErrorPixels + rmsTolerance)))
        {
            return directionDelta < 0.0;
        }
    }

    if (useNarrowGuidedMatchCap)
    {
        const double candidateMagnitudeSupport = narrowGuidedMagnitudePriorityScore(candidate);
        const double bestMagnitudeSupport = narrowGuidedMagnitudePriorityScore(best);
        const double magnitudeSupportDelta = candidateMagnitudeSupport - bestMagnitudeSupport;
        const double meaningfulMagnitudeSupportDelta = std::max(
            2.5,
            0.08 * std::max(candidateMagnitudeSupport, bestMagnitudeSupport));
        const double rmsTolerance = std::max(2.5, static_cast<double>(settings.m_plateSolveFinalMatchRadius) * 0.12);
        const bool candidateBrightnessAccepted = hasAcceptableGuidedFinalBrightnessConsistency(settings, candidate);
        const bool bestBrightnessAccepted = hasAcceptableGuidedFinalBrightnessConsistency(settings, best);
        if ((std::fabs(magnitudeSupportDelta) >= meaningfulMagnitudeSupportDelta)
            && (candidate.rmsErrorPixels <= (best.rmsErrorPixels + rmsTolerance))
            && (best.rmsErrorPixels <= (candidate.rmsErrorPixels + rmsTolerance))
            && candidateBrightnessAccepted
            && bestBrightnessAccepted)
        {
            return magnitudeSupportDelta > 0.0;
        }

        const double projectedCoverageDelta = candidate.brightProjectedMatchFraction - best.brightProjectedMatchFraction;
        if ((candidate.brightProjectedStars >= 2)
            && (best.brightProjectedStars >= 2)
            && narrowGuidedActualMatchCountsAreClose
            && (std::fabs(projectedCoverageDelta) >= 0.20))
        {
            return projectedCoverageDelta > 0.0;
        }

        const double magnitudeErrorDelta = candidate.brightDetectionMagnitudeError - best.brightDetectionMagnitudeError;
        if ((candidate.brightDetections >= 3)
            && (best.brightDetections >= 3)
            && narrowGuidedActualMatchCountsAreClose
            && (std::fabs(magnitudeErrorDelta) >= 0.25))
        {
            return magnitudeErrorDelta < 0.0;
        }

        const double coverageDelta = candidate.brightDetectionMatchFraction - best.brightDetectionMatchFraction;
        if ((candidate.brightDetections >= 3)
            && (best.brightDetections >= 3)
            && narrowGuidedActualMatchCountsAreClose
            && (std::fabs(coverageDelta) >= 0.15))
        {
            return coverageDelta > 0.0;
        }
    }

    if (isWidePlateSolveContext(settings) || (m_useDirectionSeedPreference && (isNarrowField(settings))))
    {
        if (isWidePlateSolveContext(settings))
        {
            const double projectedCoverageDelta = candidate.brightProjectedMatchFraction - best.brightProjectedMatchFraction;
            if ((candidate.brightProjectedStars >= 3)
                && (best.brightProjectedStars >= 3)
                && (std::fabs(projectedCoverageDelta) >= 0.25))
            {
                return projectedCoverageDelta > 0.0;
            }

            const double magnitudeErrorDelta = candidate.brightDetectionMagnitudeError - best.brightDetectionMagnitudeError;
            if ((candidate.brightDetections >= 3)
                && (best.brightDetections >= 3)
                && (std::fabs(magnitudeErrorDelta) >= 0.15))
            {
                return magnitudeErrorDelta < 0.0;
            }
        }
        else if (m_useDirectionSeedPreference && (isNarrowField(settings)))
        {
            const double projectedCoverageDelta = candidate.brightProjectedMatchFraction - best.brightProjectedMatchFraction;
            if ((candidate.brightProjectedStars >= 2)
                && (best.brightProjectedStars >= 2)
                && narrowGuidedActualMatchCountsAreClose
                && (std::fabs(projectedCoverageDelta) >= 0.20))
            {
                return projectedCoverageDelta > 0.0;
            }

            const double magnitudeErrorDelta = candidate.brightDetectionMagnitudeError - best.brightDetectionMagnitudeError;
            if ((candidate.brightDetections >= 3)
                && (best.brightDetections >= 3)
                && narrowGuidedActualMatchCountsAreClose
                && (std::fabs(magnitudeErrorDelta) >= 0.25))
            {
                return magnitudeErrorDelta < 0.0;
            }
        }

        const double coverageDelta = candidate.brightDetectionMatchFraction - best.brightDetectionMatchFraction;
        if ((candidate.brightDetections >= 3)
            && (best.brightDetections >= 3)
            && (!(m_useDirectionSeedPreference && (isNarrowField(settings)))
                || narrowGuidedActualMatchCountsAreClose)
            && (std::fabs(coverageDelta) >= 0.15))
        {
            return coverageDelta > 0.0;
        }
    }

    if (m_useWideCatalogMagnitudePreference || m_useDirectionSeedPreference)
    {
        const double candidateScore = cachedFinalMatchPassScore(settings, candidate);
        const double bestScore = cachedFinalMatchPassScore(settings, best);
        const double scoreScale = std::max({
            1e-9,
            std::fabs(candidateScore),
            std::fabs(bestScore)
        });
        if (std::fabs(candidateScore - bestScore) > std::max(1e-8, scoreScale * 0.08)) {
            return candidateScore > bestScore;
        }
    }

    if (isLowMagnitudeNarrowGuidedSolve(settings) && narrowGuidedActualMatchCountsAreClose)
    {
        const double candidateBrightScore = cachedNarrowGuidedBrightConsistencyScore(settings, candidate);
        const double bestBrightScore = cachedNarrowGuidedBrightConsistencyScore(settings, best);
        if (std::fabs(candidateBrightScore - bestBrightScore) > 0.35) {
            return candidateBrightScore > bestBrightScore;
        }
    }

    if (useNarrowGuidedMatchCap && (finalMatchDelta != 0)) {
        return finalMatchDelta > 0;
    }

    if (std::abs(finalMatchDelta) <= 1)
    {
        const double comparableRms = std::min(
            static_cast<double>(settings.m_plateSolveFinalMatchRadius),
            kWideFovMagnitudePreferenceMaxRmsPixels);
        if (shouldPreferBrighterCatalogMean(
                candidate.meanCatalogMagnitude,
                best.meanCatalogMagnitude,
                candidate.rmsErrorPixels,
                best.rmsErrorPixels,
                comparableRms))
        {
            return true;
        }
        if (shouldPreferBrighterCatalogMean(
                best.meanCatalogMagnitude,
                candidate.meanCatalogMagnitude,
                best.rmsErrorPixels,
                candidate.rmsErrorPixels,
                comparableRms))
        {
            return false;
        }

        if (!qFuzzyCompare(candidate.medianErrorPixels + 1.0, best.medianErrorPixels + 1.0)) {
            return candidate.medianErrorPixels < best.medianErrorPixels;
        }
        if (!qFuzzyCompare(candidate.rmsErrorPixels + 1.0, best.rmsErrorPixels + 1.0)) {
            return candidate.rmsErrorPixels < best.rmsErrorPixels;
        }
        if (!qFuzzyCompare(candidate.brightnessRankError + 1.0, best.brightnessRankError + 1.0)) {
            return candidate.brightnessRankError < best.brightnessRankError;
        }
    }

    if (finalMatchDelta != 0) {
        return finalMatchDelta > 0;
    }
    if (candidate.rawMatchCount != best.rawMatchCount) {
        return candidate.rawMatchCount > best.rawMatchCount;
    }

    return isBetterWeakModeRefinedEvaluation(candidate.pose, best.pose);
}

bool CameraPlateSolver::SolverContext::hasCompetitiveRollAlias(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const FinalMatchPassEvaluation& winner, double finalMatchRadius, QString *reason, FinalMatchPassEvaluation *betterAlias)
{
    if (!m_useDirectionSeedPreference
        || m_directionSeedHasRollPreference
        || !winner.projectorValid
        || (!isNarrowField(settings))
        || (winner.finalMatches.size() < settings.m_plateSolveMinMatches)
        || !std::isfinite(winner.rmsErrorPixels))
    {
        return false;
    }

    const int competitiveMatchCount = std::max(
        settings.m_plateSolveMinMatches,
        static_cast<int>(std::ceil(static_cast<double>(winner.finalMatches.size()) * 0.85)));
    const double maxCompetitiveRms = std::max(
        winner.rmsErrorPixels + 2.0,
        winner.rmsErrorPixels * 1.15);
    const double maxCompetitiveWorstError = std::max(
        winner.maxErrorPixels + 4.0,
        winner.maxErrorPixels * 1.15);
    const std::array<double, 12> rollOffsets = {{
        -30.0, 30.0,
        -60.0, 60.0,
        -90.0, 90.0,
        -120.0, 120.0,
        -150.0, 150.0,
        180.0, -180.0
    }};
    const bool winnerHasStrongSparseAnchors =
        hasHighConfidenceSparseGuidedAnchors(settings, winner);
    const double winnerSeedConsistency =
        narrowGuidedSeedConsistencyScore(settings, winner);
    const double winnerBrightConsistency =
        narrowGuidedBrightConsistencyScore(settings, winner);
    const double winnerFinalScore = std::max(
        finalMatchPassScore(settings, winner),
        finalMatchPassEvidenceScore(settings, winner));
    const double winnerLogOdds = poseFalseAlarmLogOdds(
        catalogContext, winner, imageSize, finalMatchRadius, static_cast<int>(starDetections.size()));

    // Adoption: if a roll alias matches the rare *bright* stars far better than the
    // winner (much higher bright-weighted log-odds), it is almost certainly the true
    // pose and the winner is a faint-coincidence look-alike. Rather than reject as
    // ambiguous, hand that alias back so the caller adopts it. Tracked across all roll
    // offsets so the best such alias wins, and preferred over an ambiguity rejection.
    constexpr double kRollAdoptLogOddsMargin = 15.0;
    FinalMatchPassEvaluation adoptableAlias;
    double adoptableAliasLogOdds = -std::numeric_limits<double>::infinity();
    bool haveAdoptableAlias = false;
    bool ambiguousAliasFound = false;

    for (double rollOffset : rollOffsets)
    {
        Evaluation competitorPose = winner.pose;
        competitorPose.anchored = false;
        competitorPose.sparseGuidedPair = false;
        competitorPose.guidedTriangle = false;
        competitorPose.anchorDetectionIndex = -1;
        competitorPose.anchorCatalogIndex = -1;
        competitorPose.secondaryAnchorDetectionIndex = -1;
        competitorPose.secondaryAnchorCatalogIndex = -1;
        competitorPose.tertiaryAnchorDetectionIndex = -1;
        competitorPose.tertiaryAnchorCatalogIndex = -1;
        competitorPose.rollDegrees = normalizeDegrees(winner.pose.rollDegrees + rollOffset);
        competitorPose.matches.clear();
        competitorPose.matchCount = 0;
        competitorPose.rmsErrorPixels = std::numeric_limits<double>::infinity();

        if (angularDistanceDegrees(competitorPose.rollDegrees, winner.pose.rollDegrees) < 1.0) {
            continue;
        }

        const FinalMatchPassEvaluation competitor = evaluateFinalMatchPass(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            detectionIndices,
            competitorPose,
            finalMatchRadius);
        if (!competitor.projectorValid
            || (competitor.finalMatches.size() < competitiveMatchCount)
            || !std::isfinite(competitor.rmsErrorPixels)
            || (competitor.rmsErrorPixels > maxCompetitiveRms)
            || (competitor.maxErrorPixels > maxCompetitiveWorstError)
            || !hasGeometricallyConsistentMatches(
                starDetections,
                competitor.projectedStars,
                competitor.finalMatches,
                finalMatchRadius)
            || !isAcceptableDirectionSeedSolve(
                settings,
                starDetections,
                competitor.finalMatches,
                competitor.rmsErrorPixels,
                competitor.maxErrorPixels)
            || !hasAcceptableGuidedFinalBrightnessConsistency(settings, competitor))
        {
            continue;
        }

        if (winnerHasStrongSparseAnchors
            && !hasHighConfidenceSparseGuidedAnchors(settings, competitor))
        {
            continue;
        }

        // Bright-weighted log-odds tie-break. Roll ambiguity in deep fields is a
        // false-alarm problem: many rolls match hundreds of *faint* stars at similar
        // count/RMS, so those statistics cannot tell the correct roll apart. The
        // false-alarm log-odds weights each match by the rarity of its catalog star
        // (-log density), so the rare *bright* stars dominate -- whichever roll lands
        // them on real detections wins decisively. If the winner's log-odds clearly
        // beats this alias, the alias is a faint-coincidence look-alike, not a genuine
        // competitor, so dismiss it. This overrides the count/RMS-based
        // aliasIsStrictlyBetterFit below, which is precisely what gets fooled in deep
        // fields (e.g. galaxy-m101 @maxMag15). It cannot create a false positive: a
        // wrong winner matching only faint coincidences has *lower* log-odds than a
        // correct bright-matching alias, so it never wins this comparison.
        const double competitorLogOdds = poseFalseAlarmLogOdds(
            catalogContext, competitor, imageSize, finalMatchRadius, static_cast<int>(starDetections.size()));
        constexpr double kRollLogOddsMargin = 5.0;
        if (winnerLogOdds >= (competitorLogOdds + kRollLogOddsMargin))
        {
            qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: ignoring roll alias with weaker bright-weighted log-odds"
                     << "winnerRoll" << winner.pose.rollDegrees << "winnerLogOdds" << winnerLogOdds
                     << "aliasRoll" << competitor.pose.rollDegrees << "aliasLogOdds" << competitorLogOdds;
            continue;
        }
        // Adopt direction: this alias matches the rare bright stars far better than the
        // winner -> it is the true pose. Record the best such alias (do not reject).
        if (competitorLogOdds >= (winnerLogOdds + kRollAdoptLogOddsMargin))
        {
            if (!haveAdoptableAlias || (competitorLogOdds > adoptableAliasLogOdds))
            {
                adoptableAlias = competitor;
                adoptableAliasLogOdds = competitorLogOdds;
                haveAdoptableAlias = true;
            }
            continue;
        }

        const double competitorSeedConsistency =
            narrowGuidedSeedConsistencyScore(settings, competitor);
        const double competitorBrightConsistency =
            narrowGuidedBrightConsistencyScore(settings, competitor);
        const double competitorFinalScore = std::max(
            finalMatchPassScore(settings, competitor),
            finalMatchPassEvidenceScore(settings, competitor));
        const double seedConsistencyDelta =
            winnerSeedConsistency - competitorSeedConsistency;
        const double brightConsistencyDelta =
            winnerBrightConsistency - competitorBrightConsistency;
        const auto winnerScoreAtLeastFraction = [&](double fraction) {
            if ((std::fabs(winnerFinalScore) < 1e-9)
                && (std::fabs(competitorFinalScore) < 1e-9))
            {
                return true;
            }
            return winnerFinalScore >= (competitorFinalScore * fraction);
        };
        const auto winnerScoreStrictlyBetter = [&](double factor) {
            if ((std::fabs(winnerFinalScore) < 1e-9)
                && (std::fabs(competitorFinalScore) < 1e-9))
            {
                return false;
            }
            return winnerFinalScore >= (competitorFinalScore * factor);
        };

        // When the roll alias is a strictly better geometric fit than the winner --
        // a comparable number of final matches but a meaningfully lower RMS -- the
        // winner is almost certainly the wrong pose, and the soft seed-consistency /
        // brightness / final-score heuristics below must not be allowed to dismiss the
        // alias. Doing so returns a confident wrong-roll pose: e.g. galaxy-m101 at
        // maxMag 16 returns roll ~-2.6 deg (123 matches, RMS 16.23) when the true roll
        // is ~+87 deg (120 matches, RMS 15.80) -- the alias matches nearly as many
        // stars at a clearly lower RMS, yet scores higher on the seed-consistency
        // heuristic, so the dismissal accepts the wrong pose.
        //
        // The "strictly lower RMS" requirement is what keeps this from over-rejecting
        // correct solves in ultra-dense fields. There, coincidental roll aliases can
        // tie the winner on match count, but the correct winner still has the lowest
        // RMS (e.g. galaxy-m31: winner RMS 15.87 vs nearest alias 15.95), so the guard
        // does not fire and the existing seed-consistency dismissals still apply.
        const double kAliasRmsImprovementMargin = 0.25;
        const bool aliasIsStrictlyBetterFit =
            std::isfinite(competitor.rmsErrorPixels)
            && (competitor.finalMatches.size()
                >= static_cast<qsizetype>(std::floor(static_cast<double>(winner.finalMatches.size()) * 0.95)))
            && (competitor.rmsErrorPixels <= (winner.rmsErrorPixels - kAliasRmsImprovementMargin));

        if (!plateSolveStartUsesRoll(settings) && !aliasIsStrictlyBetterFit)
        {
            const double seedConsistencyMargin =
                winner.pose.sparseGuidedPair ? 0.75 : 1.5;
            if ((seedConsistencyDelta >= seedConsistencyMargin)
                && winnerScoreAtLeastFraction(0.85))
            {
                qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: ignoring roll alias with weaker seed consistency"
                         << "winnerRoll" << winner.pose.rollDegrees
                         << "winnerMatches" << winner.finalMatches.size()
                         << "winnerRms" << winner.rmsErrorPixels
                         << "winnerSeed" << winnerSeedConsistency
                         << "winnerScore" << winnerFinalScore
                         << "aliasRoll" << competitor.pose.rollDegrees
                         << "aliasMatches" << competitor.finalMatches.size()
                         << "aliasRms" << competitor.rmsErrorPixels
                         << "aliasSeed" << competitorSeedConsistency
                         << "aliasScore" << competitorFinalScore;
                continue;
            }

            const double seedProjectedSupportDelta =
                winner.matchedSeedProjectedMagnitudeSupport - competitor.matchedSeedProjectedMagnitudeSupport;
            const double seedProjectedFractionDelta =
                winner.seedProjectedMagnitudeMatchFraction - competitor.seedProjectedMagnitudeMatchFraction;
            if (((seedProjectedSupportDelta >= std::max(1.5, 0.12 * std::max(
                        winner.matchedSeedProjectedMagnitudeSupport,
                        competitor.matchedSeedProjectedMagnitudeSupport)))
                    || (seedProjectedFractionDelta >= 0.035))
                && winnerScoreAtLeastFraction(0.88))
            {
                qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: ignoring roll alias with weaker seed-projected consistency"
                         << "winnerRoll" << winner.pose.rollDegrees
                         << "winnerMatches" << winner.finalMatches.size()
                         << "winnerRms" << winner.rmsErrorPixels
                         << "winnerSeedProjected" << winner.matchedSeedProjectedMagnitudeSupport
                         << "winnerSeedProjectedFraction" << winner.seedProjectedMagnitudeMatchFraction
                         << "winnerScore" << winnerFinalScore
                         << "aliasRoll" << competitor.pose.rollDegrees
                         << "aliasMatches" << competitor.finalMatches.size()
                         << "aliasRms" << competitor.rmsErrorPixels
                         << "aliasSeedProjected" << competitor.matchedSeedProjectedMagnitudeSupport
                         << "aliasSeedProjectedFraction" << competitor.seedProjectedMagnitudeMatchFraction
                         << "aliasScore" << competitorFinalScore;
                continue;
            }

            if ((brightConsistencyDelta >= 1.0)
                && winnerScoreAtLeastFraction(0.90))
            {
                qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: ignoring roll alias with weaker brightness consistency"
                         << "winnerRoll" << winner.pose.rollDegrees
                         << "winnerMatches" << winner.finalMatches.size()
                         << "winnerRms" << winner.rmsErrorPixels
                         << "winnerBright" << winnerBrightConsistency
                         << "winnerScore" << winnerFinalScore
                         << "aliasRoll" << competitor.pose.rollDegrees
                         << "aliasMatches" << competitor.finalMatches.size()
                         << "aliasRms" << competitor.rmsErrorPixels
                         << "aliasBright" << competitorBrightConsistency
                         << "aliasScore" << competitorFinalScore;
                continue;
            }

            if (winnerScoreStrictlyBetter(1.12))
            {
                qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: ignoring roll alias with weaker final score"
                         << "winnerRoll" << winner.pose.rollDegrees
                         << "winnerMatches" << winner.finalMatches.size()
                         << "winnerRms" << winner.rmsErrorPixels
                         << "winnerScore" << winnerFinalScore
                         << "aliasRoll" << competitor.pose.rollDegrees
                         << "aliasMatches" << competitor.finalMatches.size()
                         << "aliasRms" << competitor.rmsErrorPixels
                         << "aliasScore" << competitorFinalScore;
                continue;
            }
        }

        if (reason)
        {
            *reason = QStringLiteral("roll alias at %1 deg: matches=%2/%3 RMS=%4/%5 seed=%6/%7 score=%8/%9")
                .arg(competitor.pose.rollDegrees, 0, 'f', 2)
                .arg(competitor.finalMatches.size())
                .arg(winner.finalMatches.size())
                .arg(competitor.rmsErrorPixels, 0, 'f', 2)
                .arg(winner.rmsErrorPixels, 0, 'f', 2)
                .arg(competitorSeedConsistency, 0, 'f', 2)
                .arg(winnerSeedConsistency, 0, 'f', 2)
                .arg(competitorFinalScore, 0, 'f', 2)
                .arg(winnerFinalScore, 0, 'f', 2);
        }
        qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: rejecting ambiguous direction-seeded solution"
                 << "winnerRoll" << winner.pose.rollDegrees
                 << "winnerMatches" << winner.finalMatches.size()
                 << "winnerRms" << winner.rmsErrorPixels
                 << "winnerSeed" << winnerSeedConsistency
                 << "winnerBright" << winnerBrightConsistency
                 << "winnerScore" << winnerFinalScore
                 << "aliasRoll" << competitor.pose.rollDegrees
                 << "aliasMatches" << competitor.finalMatches.size()
                 << "aliasRms" << competitor.rmsErrorPixels
                 << "aliasSeed" << competitorSeedConsistency
                 << "aliasBright" << competitorBrightConsistency
                 << "aliasScore" << competitorFinalScore;
        // Genuinely ambiguous on count/RMS. Don't reject yet -- keep scanning, because
        // a later roll offset may be an adoptable bright-better alias, which is the
        // correct resolution and takes precedence over an ambiguity rejection.
        ambiguousAliasFound = true;
    }

    // A bright-better alias decisively wins (matches the rare bright stars) -> adopt it
    // rather than rejecting. Takes precedence over a count/RMS ambiguity.
    if (haveAdoptableAlias)
    {
        if (betterAlias) {
            *betterAlias = adoptableAlias;
        }
        qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: adopting bright-better roll alias"
                 << "winnerRoll" << winner.pose.rollDegrees << "winnerLogOdds" << winnerLogOdds
                 << "aliasRoll" << adoptableAlias.pose.rollDegrees
                 << "aliasLogOdds" << adoptableAliasLogOdds
                 << "aliasMatches" << adoptableAlias.finalMatches.size();
        return false;
    }

    return ambiguousAliasFound;
}

bool CameraPlateSolver::SolverContext::isBetterEvaluationForMode(const Evaluation& candidate, const Evaluation& best, bool useWeakModeScoring, bool useGuidedDirectionScoring)
{
    if (useGuidedDirectionScoring) {
        return isBetterGuidedDirectionEvaluation(candidate, best);
    }
    if (useWeakModeScoring) {
        return isBetterWeakModeEvaluation(candidate, best);
    }
    return isBetterEvaluation(candidate, best);
}

double CameraPlateSolver::SolverContext::guidedAnchorEvaluationScore(const Evaluation& evaluation, double matchRadiusPixels)
{
    if (!evaluation.valid || evaluation.matches.isEmpty()) {
        return -std::numeric_limits<double>::infinity();
    }

    const double anchorDistance = evaluation.matches.first().distancePixels;
    const double safeRadius = std::max(1.0, matchRadiusPixels);
    const double anchorRadius = std::max(4.0, std::min(24.0, safeRadius));
    const double anchorAffinity = 1.0 / (1.0 + std::pow(anchorDistance / anchorRadius, 2.0));
    const double rmsAffinity = evaluationRmsQuality(evaluation, safeRadius);

    return (wideEvaluationMatchWeight(evaluation) * 0.6 + 6.0 * anchorAffinity)
        * rmsAffinity
        * brightnessAffinity(evaluation)
        * catalogMagnitudeAffinity(evaluation)
        * directionSeedAffinity(evaluation)
        * fovSeedAffinity(evaluation);
}

double CameraPlateSolver::SolverContext::guidedAnchorSearchScore(const Evaluation& evaluation, const GuidedAnchorPair& anchor, double matchRadiusPixels)
{
    if (!evaluation.valid) {
        return -std::numeric_limits<double>::infinity();
    }

    const double directPenalty = std::pow(anchor.initialDistancePixels / 48.0, 2.0);
    const double radialPenalty = std::pow(anchor.radialErrorPixels / 96.0, 2.0);
    const double reliabilityBonus = std::min(1.5, std::log1p(anchor.detectionReliability) * 0.15);
    return guidedAnchorEvaluationScore(evaluation, matchRadiusPixels)
        - directPenalty
        - radialPenalty
        + reliabilityBonus;
}

bool CameraPlateSolver::SolverContext::isBetterGuidedAnchorEvaluation(const Evaluation& candidate, const Evaluation& best, double matchRadiusPixels)
{
    if (!candidate.valid) {
        return false;
    }
    if (!best.valid) {
        return true;
    }

    const double candidateScore = guidedAnchorEvaluationScore(candidate, matchRadiusPixels);
    const double bestScore = guidedAnchorEvaluationScore(best, matchRadiusPixels);
    if (std::fabs(candidateScore - bestScore) > 0.05) {
        return candidateScore > bestScore;
    }

    const double candidateAnchorDistance = candidate.matches.isEmpty()
        ? std::numeric_limits<double>::infinity()
        : candidate.matches.first().distancePixels;
    const double bestAnchorDistance = best.matches.isEmpty()
        ? std::numeric_limits<double>::infinity()
        : best.matches.first().distancePixels;
    const double anchorRadius = std::max(4.0, std::min(24.0, matchRadiusPixels));
    const bool candidateAnchorTight = candidateAnchorDistance <= anchorRadius;
    const bool bestAnchorTight = bestAnchorDistance <= anchorRadius;
    if (candidateAnchorTight != bestAnchorTight) {
        return candidateAnchorTight;
    }
    if (!qFuzzyCompare(candidateAnchorDistance + 1.0, bestAnchorDistance + 1.0)) {
        return candidateAnchorDistance < bestAnchorDistance;
    }
    if (candidate.matchCount != best.matchCount) {
        return candidate.matchCount > best.matchCount;
    }
    if (!qFuzzyCompare(candidate.rmsErrorPixels + 1.0, best.rmsErrorPixels + 1.0)) {
        return candidate.rmsErrorPixels < best.rmsErrorPixels;
    }
    return candidate.meanCatalogMagnitude < best.meanCatalogMagnitude;
}

CameraPlateSolver::SolverContext::Evaluation CameraPlateSolver::SolverContext::searchGuidedAnchorPose(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, QVector<Evaluation> *candidatePool)
{
    Evaluation best;
    const bool useStartFov = plateSolveStartUsesFov(settings);
    const bool useStartDirection = plateSolveStartUsesDirection(settings);
    const bool useStartRoll = plateSolveStartUsesRoll(settings);
    const bool useStartLens = plateSolveStartUsesLens(settings);
    const bool useWidePlateSolve = isWidePlateSolveContext(settings);
    const bool useFaintNarrowAnchors = useStartDirection
        && !useWidePlateSolve
        && (isNarrowField(settings));
    const bool useDenseWideGuidedDirection = useStartDirection
        && useWidePlateSolve
        && (starDetections.size() > 32);
    const bool useWideWeakAnchorSearch = !plateSolveStartUsesDirection(settings)
        && useWidePlateSolve;
    if ((!useStartDirection && !useWideWeakAnchorSearch)
        || starDetections.isEmpty()
        || catalogContext.visibleStars.isEmpty())
    {
        return best;
    }

    const double localRadiusDegrees = std::max(
        static_cast<double>(settings.m_plateSolveAzElSearchRadius) + static_cast<double>(settings.m_fov) * 2.0,
        static_cast<double>(settings.m_fov) * 4.0);
    const QVector<VisibleCatalogStar> localVisibleStars = selectLocalVisibleStars(
        catalogContext.visibleStars,
        settings.m_azimuth,
        settings.m_elevation,
        localRadiusDegrees,
        2048);
    if (localVisibleStars.isEmpty()) {
        return best;
    }

    const QVector<GuidedAnchorPair> anchors = findGuidedAnchorPairs(
        settings,
        catalogContext,
        imageSize,
        starDetections,
        localVisibleStars);
    if (anchors.isEmpty()) {
        return best;
    }

    QVector<int> allowedCatalogIndices;
    allowedCatalogIndices.reserve(localVisibleStars.size());
    for (const VisibleCatalogStar& star : localVisibleStars) {
        allowedCatalogIndices.append(star.catalogIndex);
    }

    const double fixedCenterOffsetX = useStartLens ? settings.m_lensCenterOffsetX : 0.0;
    const double fixedCenterOffsetY = useStartLens ? settings.m_lensCenterOffsetY : 0.0;
    const double fixedDistortionK1 = useStartLens ? settings.m_lensDistortionK1 : 0.0;
    const bool calibrateLens = canCalibrateLens(settings);
    QVector<QPointF> centerOffsetSeeds;
    centerOffsetSeeds.append(QPointF(fixedCenterOffsetX, fixedCenterOffsetY));
    QVector<double> distortionSeeds;
    distortionSeeds.append(fixedDistortionK1);
    if (calibrateLens && useWidePlateSolve)
    {
        const double xStep = imageSize.width() * 0.08;
        const double yStep = imageSize.height() * 0.08;
        const std::array<QPointF, 4> coarseOffsets = {{
            QPointF(-xStep, 0.0),
            QPointF(xStep, 0.0),
            QPointF(0.0, -yStep),
            QPointF(0.0, yStep)
        }};
        for (const QPointF& offset : coarseOffsets) {
            centerOffsetSeeds.append(centerOffsetSeeds.first() + offset);
        }
    }
    const double finalMatchRadius = std::max(1.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius));
    const double anchorMatchRadiusMultiplier = (useStartDirection && useWidePlateSolve)
        ? 3.0
        : 4.0;
    const double anchorMatchRadius = std::max(finalMatchRadius,
        std::min(128.0, std::max(finalMatchRadius, static_cast<double>(settings.m_plateSolveMatchRadius)) * anchorMatchRadiusMultiplier));
    const QVector<double> primaryRollSeedOffsets = useStartRoll
        ? QVector<double>{0.0, -5.0, 5.0, -10.0, 10.0}
        : QVector<double>{0.0, -5.0, 5.0, -10.0, 10.0, -20.0, 20.0, -35.0, 35.0};
    const QVector<double> expandedRollSeedOffsets = {
        -30.0, 30.0, -45.0, 45.0, -60.0, 60.0, -90.0, 90.0,
        -120.0, 120.0, -150.0, 150.0, -180.0, 180.0
    };
    // Direction-seeded anchor refinement already sweeps FoV locally from the 1.0x seed,
    // so extra scale seeds only multiply the expensive anchor search.
    const QVector<double> fovSeedScales = useStartDirection
        ? QVector<double>{1.0}
        : (useStartFov && useWideWeakAnchorSearch)
            ? QVector<double>{0.96, 1.0, 1.04}
            : QVector<double>{0.88, 0.96, 1.0, 1.04, 1.12};
    const int anchorLimit = std::min(
        useWideWeakAnchorSearch ? 24
            : useFaintNarrowAnchors ? 48
            : useDenseWideGuidedDirection ? 8
            : (useStartDirection && useWidePlateSolve && (starDetections.size() <= 16)) ? 4
            : 12,
        static_cast<int>(anchors.size()));
    const int expandedRollMinMatches = std::max(3, settings.m_plateSolveMinMatches - 1);
    double bestSearchScore = -std::numeric_limits<double>::infinity();

    if (kLogPlateSolveCandidates)
    {
        qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: guided anchor search"
                 << "anchors" << anchors.size()
                 << "using" << anchorLimit
                 << "localStars" << localVisibleStars.size()
                 << "radius" << localRadiusDegrees;
        for (int i = 0; i < anchorLimit; ++i)
        {
            const GuidedAnchorPair& anchor = anchors[i];
            const QString anchorName = ((anchor.catalogIndex >= 0) && (anchor.catalogIndex < catalogContext.catalogStars.size()))
                ? catalogDisplayName(catalogContext.catalogStars[anchor.catalogIndex])
                : QString();
            qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: guided anchor candidate"
                     << i
                     << anchorName
                     << "catalog" << anchor.catalogIndex
                     << "detection" << anchor.detectionIndex
                     << "mag" << anchor.magnitude
                     << "radial" << anchor.radialErrorPixels
                     << "direct" << anchor.initialDistancePixels
                     << "roll" << anchor.estimatedRollDegrees;
        }
    }

    auto evaluateRollSeedOffsets = [&](const QVector<double>& rollSeedOffsets)
    {
        for (int anchorIndex = 0; anchorIndex < anchorLimit; ++anchorIndex)
        {
            const GuidedAnchorPair& anchor = anchors[anchorIndex];
            const auto anchorVisibleIt = catalogContext.visibleStarIndexByCatalogIndex.constFind(anchor.catalogIndex);
            if (anchorVisibleIt == catalogContext.visibleStarIndexByCatalogIndex.cend()) {
                continue;
            }
            const SkyVector anchorVector = catalogContext.visibleStars[*anchorVisibleIt].vector;
            QVector<int> anchoredDetectionIndices = detectionIndices;
            if (!anchoredDetectionIndices.contains(anchor.detectionIndex)) {
                anchoredDetectionIndices.append(anchor.detectionIndex);
            }

            for (double fovScale : fovSeedScales)
            {
                for (double rollSeedOffset : rollSeedOffsets)
                {
                    const double seedFov = std::clamp(static_cast<double>(settings.m_fov) * fovScale,
                        static_cast<double>(CameraSettings::m_minFov),
                        static_cast<double>(CameraSettings::m_maxFov));
                    const double baseSeedRoll = anchor.estimatedRollDegrees + rollSeedOffset;

                    auto evaluateAnchorSeed = [&](double seedAzimuth,
                                                  double seedElevation,
                                                  double seedRoll,
                                                  double seedCenterOffsetX,
                                                  double seedCenterOffsetY,
                                                  double seedDistortionK1)
                    {
                        Evaluation localBest = evaluateAnchoredPose(
                            settings,
                            catalogContext,
                            imageSize,
                            captureDateTimeUtc,
                            starDetections,
                            anchoredDetectionIndices,
                            allowedCatalogIndices,
                            anchor,
                            seedAzimuth,
                            seedElevation,
                            seedRoll,
                            seedFov,
                            seedCenterOffsetX,
                            seedCenterOffsetY,
                            seedDistortionK1,
                            anchorMatchRadius);
                        if (!localBest.valid) {
                            return;
                        }

                        if (!useWideWeakAnchorSearch || (localBest.matchCount >= 2))
                        {
                            localBest = refineGuidedAnchorSeedWithLm(
                                settings,
                                catalogContext,
                                imageSize,
                                captureDateTimeUtc,
                                starDetections,
                                anchoredDetectionIndices,
                                allowedCatalogIndices,
                                anchor,
                                localBest,
                                seedCenterOffsetX,
                                seedCenterOffsetY,
                                seedDistortionK1,
                                anchorMatchRadius,
                                !isNarrowField(settings));
                        }
                        logPlateSolveEvaluation("guided-anchor", localBest);
                        if (candidatePool && localBest.valid && (localBest.matchCount >= 2)) {
                            candidatePool->append(localBest);
                        }
                        const double localSearchScore = guidedAnchorSearchScore(localBest, anchor, anchorMatchRadius);
                        if ((localSearchScore > bestSearchScore)
                            || (std::fabs(localSearchScore - bestSearchScore) <= 0.05
                                && isBetterGuidedAnchorEvaluation(localBest, best, anchorMatchRadius)))
                        {
                            bestSearchScore = localSearchScore;
                            best = localBest;
                            logPlateSolveEvaluation("guided-anchor", best, true);
                        }
                    };

                    const bool tryCoarseLensSeeds = (anchorIndex < 6)
                        && (anchor.magnitude <= 3.0)
                        && calibrateLens
                        && useWidePlateSolve
                        && useStartLens;
                    const int centerSeedCount = tryCoarseLensSeeds ? centerOffsetSeeds.size() : 1;
                    for (int centerSeedIndex = 0; centerSeedIndex < centerSeedCount; ++centerSeedIndex)
                    {
                        const QPointF centerOffsetSeed = centerOffsetSeeds[centerSeedIndex];
                        for (double distortionSeed : distortionSeeds)
                        {
                            if (centerSeedIndex == 0)
                            {
                                evaluateAnchorSeed(
                                    settings.m_azimuth,
                                    settings.m_elevation,
                                    baseSeedRoll,
                                    centerOffsetSeed.x(),
                                    centerOffsetSeed.y(),
                                    distortionSeed);
                            }

                            double alignedAzimuth = settings.m_azimuth;
                            double alignedElevation = settings.m_elevation;
                            double alignedRoll = baseSeedRoll;
                            if (anchorAlignedPoseFromPixel(
                                    settings,
                                    imageSize,
                                    starDetections[anchor.detectionIndex].m_center,
                                    anchorVector,
                                    settings.m_azimuth,
                                    settings.m_elevation,
                                    baseSeedRoll,
                                    seedFov,
                                    centerOffsetSeed.x(),
                                    centerOffsetSeed.y(),
                                    distortionSeed,
                                    alignedAzimuth,
                                    alignedElevation,
                                    alignedRoll))
                            {
                                evaluateAnchorSeed(
                                    alignedAzimuth,
                                    alignedElevation,
                                    alignedRoll,
                                    centerOffsetSeed.x(),
                                    centerOffsetSeed.y(),
                                    distortionSeed);
                            }
                        }
                    }
                }
            }
        }
    };

    evaluateRollSeedOffsets(primaryRollSeedOffsets);
    const bool denseNarrowDirectionSolve = useStartDirection
        && !useStartRoll
        && !useWidePlateSolve
        && (isNarrowField(settings))
        && (starDetections.size() > kMaxDetectionsForSolve * 2);
    if (!best.valid || (best.matchCount < expandedRollMinMatches) || denseNarrowDirectionSolve) {
        evaluateRollSeedOffsets(expandedRollSeedOffsets);
    }

    return best;
}

CameraPlateSolver::SolverContext::FinalMatchPassEvaluation CameraPlateSolver::SolverContext::searchBrightAnchorVerifierRescue(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, double finalMatchRadius)
{
    FinalMatchPassEvaluation best;
    if (starDetections.isEmpty() || catalogContext.visibleStars.isEmpty()) {
        return best;
    }

    // Brightest plausible detections: prefer saturated, then by flux; skip hot-pixel
    // suspects (the same disqualifier findGuidedAnchorPairs applies to anchor candidates).
    QVector<int> brightDetectionIndices;
    {
        QVector<int> candidates;
        candidates.reserve(starDetections.size());
        for (int i = 0; i < starDetections.size(); ++i) {
            if (!starDetections[i].m_hotPixelSuspect) {
                candidates.append(i);
            }
        }
        std::sort(candidates.begin(), candidates.end(), [&starDetections](int lhs, int rhs) {
            const CameraPipelineStarDetection& lhsDetection = starDetections[lhs];
            const CameraPipelineStarDetection& rhsDetection = starDetections[rhs];
            if (lhsDetection.m_saturated != rhsDetection.m_saturated) {
                return lhsDetection.m_saturated;
            }
            return lhsDetection.m_flux > rhsDetection.m_flux;
        });
        constexpr int kMaxRescueBrightDetections = 4;
        for (int i = 0; (i < candidates.size()) && (brightDetectionIndices.size() < kMaxRescueBrightDetections); ++i) {
            brightDetectionIndices.append(candidates[i]);
        }
    }
    if (brightDetectionIndices.isEmpty()) {
        return best;
    }

    const double localRadiusDegrees = std::max(
        static_cast<double>(settings.m_plateSolveAzElSearchRadius) + static_cast<double>(settings.m_fov) * 2.0,
        static_cast<double>(settings.m_fov) * 4.0);
    const QVector<VisibleCatalogStar> localVisibleStars = selectLocalVisibleStars(
        catalogContext.visibleStars,
        settings.m_azimuth,
        settings.m_elevation,
        localRadiusDegrees,
        2048);
    if (localVisibleStars.isEmpty()) {
        return best;
    }

    QVector<VisibleCatalogStar> brightCatalogStars;
    constexpr int kMaxRescueBrightCatalogStars = 10;
    for (const VisibleCatalogStar& star : localVisibleStars)
    {
        if ((star.catalogIndex < 0)
            || (star.catalogIndex >= catalogContext.catalogStars.size())
            || (star.magnitude > kNarrowGuidedBrightCatalogMaxMagnitude))
        {
            continue;
        }
        brightCatalogStars.append(star);
        if (brightCatalogStars.size() >= kMaxRescueBrightCatalogStars) {
            break;
        }
    }
    if (brightCatalogStars.isEmpty()) {
        return best;
    }

    // For narrow fields, the top-N brightest catalog stars *globally* within the wide
    // localRadiusDegrees (multiple FOVs, used for catalog matching) are often well
    // outside the actual frame, so the fixed-size "brightest" pool above can be entirely
    // filled with stars that can never match a detection - while the star that's
    // actually the brightest object *in the image* (e.g. ngc-2403's HIP 37078 at mag 8.2,
    // vs a pool of out-of-frame mag 3-5 stars) never gets a chance as an anchor.
    // Additively append the brightest few in-frame catalog stars not already in the pool.
    // DENSE fields only (>= 512 detections): dense narrow solves face the strict
    // bright-support acceptance gates, so a wrong-roll candidate seeded from a faint
    // anchor cannot be adopted. Sparse/moderate fields are excluded deliberately - their
    // lenient low-magnitude acceptance ladder let exactly such candidates through as
    // wrong-roll false positives (synthetic corpus collapse, 2026-06-11: 99 -> 59/100).
    if (isNarrowField(settings) && (starDetections.size() >= 512))
    {
        const auto appendInFrameAnchorStars = [&](double radiusDegrees, int budget) {
            const QVector<VisibleCatalogStar> inFrameStars = selectLocalVisibleStars(
                catalogContext.visibleStars,
                settings.m_azimuth,
                settings.m_elevation,
                radiusDegrees,
                2048);
            int added = 0;
            for (const VisibleCatalogStar& star : inFrameStars)
            {
                if (added >= budget) {
                    break;
                }
                if ((star.catalogIndex < 0)
                    || (star.catalogIndex >= catalogContext.catalogStars.size())
                    || (star.magnitude > kNarrowGuidedBrightCatalogMaxMagnitude))
                {
                    continue;
                }
                const bool alreadyPresent = std::any_of(
                    brightCatalogStars.constBegin(), brightCatalogStars.constEnd(),
                    [&star](const VisibleCatalogStar& existing) {
                        return existing.catalogIndex == star.catalogIndex;
                    });
                if (alreadyPresent) {
                    continue;
                }
                brightCatalogStars.append(star);
                ++added;
            }
        };
        // Tier 1: stars inside the frame's inscribed circle are in the image at ANY roll
        // (roll is unknown here), so each is certain to have a detection counterpart —
        // these are the highest-value anchors (ngc-2403's HIP 37078 at 0.32 degrees from
        // centre is otherwise crowded out by brighter stars that sit in the wider radius
        // but outside the actual frame). Tier 2: the wider possibly-in-frame radius.
        const double frameWidthDegrees = static_cast<double>(settings.m_fov);
        const double inscribedRadiusDegrees = 0.5 * frameWidthDegrees * std::min(
            1.0,
            static_cast<double>(std::min(imageSize.width(), imageSize.height()))
                / static_cast<double>(std::max(1, imageSize.width())));
        appendInFrameAnchorStars(inscribedRadiusDegrees, 4);
        appendInFrameAnchorStars(std::min(localRadiusDegrees, frameWidthDegrees), 4);
    }

    // Diagnostic only (SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE): dump the anchor
    // candidate sets pass 1 will sweep, so it's possible to tell whether a particular
    // detection/catalog-star pairing was even considered as an anchor.
    const bool debugSparse = qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE");
    if (debugSparse)
    {
        for (int idx : brightDetectionIndices)
        {
            const CameraPipelineStarDetection& d = starDetections[idx];
            qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: searchBrightAnchorVerifierRescue brightDetection" << idx
                     << "x" << d.m_center.x() << "y" << d.m_center.y()
                     << "flux" << d.m_flux << "saturated" << d.m_saturated
                     << "hotPixel" << d.m_hotPixelSuspect;
        }
        for (const VisibleCatalogStar& star : brightCatalogStars)
        {
            const QString name = ((star.catalogIndex >= 0) && (star.catalogIndex < catalogContext.catalogStars.size()))
                ? catalogContext.catalogStars[star.catalogIndex].name
                : QString();
            qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: searchBrightAnchorVerifierRescue brightCatalogStar"
                     << "catalogIndex" << star.catalogIndex << "name" << name
                     << "mag" << star.magnitude
                     << "az" << star.azimuthDegrees << "el" << star.elevationDegrees;
        }
    }

    QVector<int> allowedCatalogIndices;
    allowedCatalogIndices.reserve(localVisibleStars.size());
    for (const VisibleCatalogStar& star : localVisibleStars) {
        allowedCatalogIndices.append(star.catalogIndex);
    }

    const bool useStartLens = plateSolveStartUsesLens(settings);
    const double seedFov = std::clamp(static_cast<double>(settings.m_fov),
        static_cast<double>(CameraSettings::m_minFov),
        static_cast<double>(CameraSettings::m_maxFov));
    const double centerOffsetX = useStartLens ? settings.m_lensCenterOffsetX : 0.0;
    const double centerOffsetY = useStartLens ? settings.m_lensCenterOffsetY : 0.0;
    const double distortionK1 = useStartLens ? settings.m_lensDistortionK1 : 0.0;
    const double anchorMatchRadius = std::max(finalMatchRadius,
        std::min(128.0, std::max(finalMatchRadius, static_cast<double>(settings.m_plateSolveMatchRadius)) * 4.0));

    QVector<double> rollSweepDegrees;
    rollSweepDegrees.reserve(36);
    for (double roll = 0.0; roll < 360.0; roll += 10.0) {
        rollSweepDegrees.append(roll);
    }

    // Pass 1 (cheap): sweep every (bright detection x bright catalog star x roll) seed -
    // evaluateAnchoredPose/refineGuidedAnchorSeedWithLm only match against the small
    // allowedCatalogIndices set, no full-catalog projection - and keep just a small,
    // basin-deduplicated shortlist ranked by the cheap guidedAnchorSearchScore. This mirrors
    // the existing guided-anchor search's own cheap-then-expensive structure: without it,
    // the combinatorial sweep (up to 4 x 10 x 36 = 1440 seeds, on every one of the outer
    // solve's retry attempts) would multiply into thousands of full catalog-projection
    // passes per image and make the suite impractically slow.
    struct RescueShortlistCandidate
    {
        Evaluation refined;
        double cheapScore = -std::numeric_limits<double>::infinity();
    };
    // Pass-2's cost scales with detections x catalog candidates, so the shortlist depth
    // can scale inversely with field density: a 45-detection sparse field affords ~24
    // verifier-ranked candidates for the same budget 5 cost on a 1178-detection dense
    // one. Sparse fields need the depth — their cheap scores are nearly flat (true and
    // wrong-roll basins within ~0.2 of each other on narrow-3), so a 5-slot global
    // shortlist is routinely crowded out by contaminated basins before
    // poseFalseAlarmLogOdds (which separates well) ever sees the true candidate.
    const int kMaxRescueShortlist = std::clamp(2400 / std::max(1, static_cast<int>(starDetections.size())), 5, 24);
    QVector<RescueShortlistCandidate> shortlist;

    for (int detectionIndex : brightDetectionIndices)
    {
        const CameraPipelineStarDetection& detection = starDetections[detectionIndex];
        QVector<int> anchoredDetectionIndices = detectionIndices;
        if (!anchoredDetectionIndices.contains(detectionIndex)) {
            anchoredDetectionIndices.append(detectionIndex);
        }

        for (const VisibleCatalogStar& catalogStar : brightCatalogStars)
        {
            GuidedAnchorPair anchor;
            anchor.detectionIndex = detectionIndex;
            anchor.catalogIndex = catalogStar.catalogIndex;
            anchor.magnitude = catalogStar.magnitude;
            anchor.detectionReliability = cachedDetectionReliabilityMetric(starDetections, detectionIndex);

            for (double rollSeed : rollSweepDegrees)
            {
                double alignedAzimuth = settings.m_azimuth;
                double alignedElevation = settings.m_elevation;
                double alignedRoll = rollSeed;
                // Rotate the seed pose so the catalog star's projected position lands
                // exactly on the chosen detection - the same alignment primitive the
                // guided-anchor search uses to turn a (detection, star, roll) triple into
                // a coherent starting pose, just without requiring a verified seed first.
                if (!anchorAlignedPoseFromPixel(
                        settings,
                        imageSize,
                        detection.m_center,
                        catalogStar.vector,
                        settings.m_azimuth,
                        settings.m_elevation,
                        rollSeed,
                        seedFov,
                        centerOffsetX,
                        centerOffsetY,
                        distortionK1,
                        alignedAzimuth,
                        alignedElevation,
                        alignedRoll))
                {
                    continue;
                }

                Evaluation seed = evaluateAnchoredPose(
                    settings,
                    catalogContext,
                    imageSize,
                    captureDateTimeUtc,
                    starDetections,
                    anchoredDetectionIndices,
                    allowedCatalogIndices,
                    anchor,
                    alignedAzimuth,
                    alignedElevation,
                    alignedRoll,
                    seedFov,
                    centerOffsetX,
                    centerOffsetY,
                    distortionK1,
                    anchorMatchRadius);
                if (!seed.valid) {
                    continue;
                }

                const Evaluation refined = refineGuidedAnchorSeedWithLm(
                    settings,
                    catalogContext,
                    imageSize,
                    captureDateTimeUtc,
                    starDetections,
                    anchoredDetectionIndices,
                    allowedCatalogIndices,
                    anchor,
                    seed,
                    centerOffsetX,
                    centerOffsetY,
                    distortionK1,
                    anchorMatchRadius,
                    !isNarrowField(settings));
                if (!refined.valid) {
                    continue;
                }

                const double cheapScore = guidedAnchorSearchScore(refined, anchor, anchorMatchRadius);
                if (!std::isfinite(cheapScore)) {
                    continue;
                }

                int basinIndex = -1;
                for (int i = 0; i < shortlist.size(); ++i) {
                    if (sameEvaluationBasin(shortlist[i].refined, refined)) {
                        basinIndex = i;
                        break;
                    }
                }
                if (basinIndex >= 0)
                {
                    if (cheapScore > shortlist[basinIndex].cheapScore) {
                        shortlist[basinIndex] = RescueShortlistCandidate{refined, cheapScore};
                    }
                }
                else
                {
                    shortlist.append(RescueShortlistCandidate{refined, cheapScore});
                }
            }
        }
    }

    std::sort(shortlist.begin(), shortlist.end(),
              [](const RescueShortlistCandidate& lhs, const RescueShortlistCandidate& rhs) {
                  return lhs.cheapScore > rhs.cheapScore;
              });
    if (shortlist.size() > kMaxRescueShortlist) {
        shortlist.resize(kMaxRescueShortlist);
    }

    // Diagnostic only (SDRANGEL_CAMERA_PLATE_SOLVER_DEBUG_SPARSE): dump the cheap-ranked
    // shortlist that pass 2 will choose between, so it's possible to tell whether a
    // particular pose (e.g. the true Az/El) was even considered, and if so, why it lost.
    if (debugSparse)
    {
        for (int i = 0; i < shortlist.size(); ++i)
        {
            const RescueShortlistCandidate& candidate = shortlist[i];
            qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: searchBrightAnchorVerifierRescue shortlist" << i
                     << "cheapScore" << candidate.cheapScore
                     << "Az" << candidate.refined.azimuthDegrees
                     << "El" << candidate.refined.elevationDegrees
                     << "Roll" << candidate.refined.rollDegrees
                     << "FoV" << candidate.refined.fovDegrees
                     << "matches" << candidate.refined.matchCount
                     << "rms" << candidate.refined.rmsErrorPixels
                     << "anchorDet" << candidate.refined.anchorDetectionIndex
                     << "anchorCat" << candidate.refined.anchorCatalogIndex;
        }
    }

    // Pass 2 (expensive): only the handful of cheap-ranked, basin-distinct survivors get
    // the full catalog-projection pass - poseFalseAlarmLogOdds remains the selection
    // criterion between them, per the chosen implementation track.
    double bestFalseAlarmLogOdds = -std::numeric_limits<double>::infinity();
    for (const RescueShortlistCandidate& candidate : shortlist)
    {
        FinalMatchPassEvaluation candidatePass = evaluateFinalMatchPass(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            detectionIndices,
            candidate.refined,
            finalMatchRadius);
        if (!candidatePass.projectorValid) {
            if (debugSparse) {
                qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: searchBrightAnchorVerifierRescue pass2 invalid-projector"
                         << "Az" << candidate.refined.azimuthDegrees
                         << "El" << candidate.refined.elevationDegrees
                         << "Roll" << candidate.refined.rollDegrees;
            }
            continue;
        }

        const double falseAlarmLogOdds = poseFalseAlarmLogOdds(
            catalogContext,
            candidatePass,
            imageSize,
            finalMatchRadius,
            static_cast<int>(starDetections.size()));
        if (debugSparse)
        {
            qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: searchBrightAnchorVerifierRescue pass2"
                     << "faLogOdds" << falseAlarmLogOdds
                     << "Az" << candidatePass.pose.azimuthDegrees
                     << "El" << candidatePass.pose.elevationDegrees
                     << "Roll" << candidatePass.pose.rollDegrees
                     << "FoV" << candidatePass.pose.fovDegrees
                     << "matches" << candidatePass.finalMatches.size()
                     << "rms" << candidatePass.rmsErrorPixels;
        }
        if (falseAlarmLogOdds > bestFalseAlarmLogOdds)
        {
            bestFalseAlarmLogOdds = falseAlarmLogOdds;
            best = candidatePass;
        }
    }

    if (debugSparse && best.projectorValid)
    {
        qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: searchBrightAnchorVerifierRescue selected"
                 << "faLogOdds" << bestFalseAlarmLogOdds
                 << "Az" << best.pose.azimuthDegrees
                 << "El" << best.pose.elevationDegrees
                 << "Roll" << best.pose.rollDegrees
                 << "FoV" << best.pose.fovDegrees
                 << "matches" << best.finalMatches.size()
                 << "rms" << best.rmsErrorPixels;
    }

    return best;
}

void CameraPlateSolver::SolverContext::insertDistinctEvaluationCandidate(QVector<Evaluation>& candidates, const Evaluation& candidate, int maxCandidates, bool useWeakModeScoring, const char *stage, int interestingMatchCount, int minPoolMatchCount, bool useGuidedDirectionScoring)
{
    if (!candidate.valid) {
        return;
    }

    // Quality floor: don't admit obvious noise into the multi-hypothesis pool. A candidate
    // must have at least 3 matches *and* an RMS well inside the acquisition radius. This is
    // intentionally based on the coarse acquisition geometry, not the much tighter final
    // acceptance radius, so rough-but-promising weak-mode basins survive long enough to be
    // rescored and refined.
    const int effectiveMinPoolMatchCount = candidate.sparseGuidedPair
        ? std::max(2, minPoolMatchCount)
        : std::max(3, minPoolMatchCount);
    if (candidate.matchCount < effectiveMinPoolMatchCount) {
        if (useWeakModeScoring && stage && (candidate.matchCount >= interestingMatchCount)) {
            logWeakModePoolDecision(stage, "reject-too-few-matches", candidate, 0.0);
        }
        return;
    }
    const double poolQualityRadius = (candidate.anchored && useGuidedDirectionScoring)
        ? std::max(2.0, m_weakModeNormalizationPixels * 4.0)
        : std::max(2.0, m_weakModeNormalizationPixels * 0.95);
    if (candidate.rmsErrorPixels > poolQualityRadius) {
        if (useWeakModeScoring && stage && (candidate.matchCount >= interestingMatchCount)) {
            logWeakModePoolDecision(stage, "reject-rms-floor", candidate, poolQualityRadius);
        }
        return;
    }

    for (Evaluation& existing : candidates)
    {
        if (sameEvaluationBasin(candidate, existing))
        {
            if (isBetterEvaluationForMode(candidate, existing, useWeakModeScoring, useGuidedDirectionScoring)) {
                if (useWeakModeScoring && stage && (candidate.matchCount >= interestingMatchCount)) {
                    logWeakModePoolDecision(stage, "replace-same-basin", candidate, poolQualityRadius, &existing);
                }
                existing = candidate;
            } else if (useWeakModeScoring && stage && (candidate.matchCount >= interestingMatchCount)) {
                logWeakModePoolDecision(stage, "keep-existing-basin", candidate, poolQualityRadius, &existing);
            }
            return;
        }
    }

    // Once the weak-mode pool is full, avoid appending/sorting candidates that cannot
    // possibly survive. This cuts a lot of blind/FoV churn without changing the winner
    // selection logic for genuinely competitive basins.
    if (useWeakModeScoring && (candidates.size() >= maxCandidates) && !candidates.isEmpty())
    {
        const Evaluation& weakestCandidate = candidates.constLast();
        if (!isBetterEvaluationForMode(candidate, weakestCandidate, useWeakModeScoring, useGuidedDirectionScoring))
        {
            const bool nearTailChallenge = (candidate.matchCount >= weakestCandidate.matchCount)
                || (candidate.rmsErrorPixels <= weakestCandidate.rmsErrorPixels);
            if (stage && nearTailChallenge && (candidate.matchCount >= interestingMatchCount)) {
                logWeakModePoolDecision(stage, "reject-below-tail", candidate, poolQualityRadius, &weakestCandidate);
            }
            return;
        }
    }

    candidates.append(candidate);
    std::sort(candidates.begin(), candidates.end(), [this, useWeakModeScoring, useGuidedDirectionScoring](const Evaluation& lhs, const Evaluation& rhs) {
        return isBetterEvaluationForMode(lhs, rhs, useWeakModeScoring, useGuidedDirectionScoring);
    });
    bool candidateKept = true;
    if (candidates.size() > maxCandidates)
    {
        QVector<Evaluation> trimmedCandidates;
        trimmedCandidates.reserve(candidates.size());
        for (const Evaluation& existing : candidates)
        {
            if (existing.sparseGuidedPair || existing.guidedTriangle) {
                trimmedCandidates.append(existing);
            }
        }
        for (const Evaluation& existing : candidates)
        {
            if (existing.sparseGuidedPair || existing.guidedTriangle) {
                continue;
            }
            if (trimmedCandidates.size() >= maxCandidates) {
                break;
            }
            trimmedCandidates.append(existing);
        }
        if (trimmedCandidates.isEmpty()) {
            candidates.resize(maxCandidates);
        } else {
            candidates = std::move(trimmedCandidates);
        }
        candidateKept = std::any_of(candidates.cbegin(), candidates.cend(), [this, &candidate](const Evaluation& existing) {
            return sameEvaluationIdentity(existing, candidate);
        });
    }
    if (useWeakModeScoring && stage && (candidate.matchCount >= interestingMatchCount)) {
        if (candidateKept) {
            logWeakModePoolDecision(stage, "add-distinct-basin", candidate, poolQualityRadius);
        } else {
            logWeakModePoolDecision(stage, "drop-pool-full", candidate, poolQualityRadius);
        }
    }
}

CameraPlateSolver::SolverContext::Evaluation CameraPlateSolver::SolverContext::rescoreWeakModeCandidateWithDistortionSweep(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const Evaluation& candidate)
{
    if (!candidate.valid) {
        return candidate;
    }

    // Preserve the user's calibrated lens centre when we can't calibrate the lens during
    // this solve; previously this function unconditionally zeroed the centre offsets,
    // throwing away a manual calibration and biasing the rescore against it. Likewise the
    // distortion sweep only makes sense when we're allowed to calibrate the lens — if the
    // user disabled calibration, a single re-evaluation with their settings is enough.
    const bool calibrate = canCalibrateLens(settings);
    const double baseCenterOffsetX = calibrate ? candidate.centerOffsetXPixels : settings.m_lensCenterOffsetX;
    const double baseCenterOffsetY = calibrate ? candidate.centerOffsetYPixels : settings.m_lensCenterOffsetY;
    const double baseDistortionK1 = calibrate ? candidate.distortionK1 : settings.m_lensDistortionK1;

    Evaluation best = candidate;
    // Sweep the distortion RELATIVE to the candidate's own K1 (baseDistortionK1), symmetrically.
    // The previous sweep used fixed ABSOLUTE values {-0.05,-0.025,0,0.025}: a candidate already
    // refined to e.g. K1=-0.09 was only re-evaluated far from its value and never above it, so the
    // rescore could not refine a strong barrel candidate around its own K1 (and baseDistortionK1
    // was unused on the calibrate path). The 0.0 delta re-evaluates at the candidate's exact K1;
    // best starts as the candidate, so the sweep can only improve on it, never regress.
    const std::array<double, 5> distortionSweep = {{-0.05, -0.025, 0.0, 0.025, 0.05}};
    for (double distortionDelta : distortionSweep)
    {
        const double distortionK1 = calibrate ? (baseDistortionK1 + distortionDelta) : baseDistortionK1;
        Evaluation rescored = evaluatePose(
            settings,
            catalogContext,
            imageSize,
            captureDateTimeUtc,
            starDetections,
            detectionIndices,
            candidate.azimuthDegrees,
            candidate.elevationDegrees,
            candidate.rollDegrees,
            candidate.fovDegrees,
            nullptr,
            baseCenterOffsetX,
            baseCenterOffsetY,
            distortionK1,
            -1.0);
        rescored.anchored = candidate.anchored;
        rescored.sparseGuidedPair = candidate.sparseGuidedPair;
        rescored.guidedTriangle = candidate.guidedTriangle;
        rescored.anchorDetectionIndex = candidate.anchorDetectionIndex;
        rescored.anchorCatalogIndex = candidate.anchorCatalogIndex;
        rescored.secondaryAnchorDetectionIndex = candidate.secondaryAnchorDetectionIndex;
        rescored.secondaryAnchorCatalogIndex = candidate.secondaryAnchorCatalogIndex;
        rescored.tertiaryAnchorDetectionIndex = candidate.tertiaryAnchorDetectionIndex;
        rescored.tertiaryAnchorCatalogIndex = candidate.tertiaryAnchorCatalogIndex;
        if (isBetterWeakModeEvaluation(rescored, best)) {
            best = rescored;
        }
        if (!calibrate) {
            // No need to sweep distortion if we can't calibrate; the loop body is identical.
            break;
        }
    }

    return best;
}

void CameraPlateSolver::SolverContext::logPlateSolveEvaluation(const char *stage, const Evaluation& evaluation, bool isNewBest, bool forceLog)
{
    if (!evaluation.valid || (!isNewBest && !kLogPlateSolveCandidates && !forceLog)) {
        return;
    }

    qCDebug(cameraPlateSolverLog).noquote().nospace()
        << "CameraPlateSolver[" << stage << "] "
        << (isNewBest ? "best " : "candidate ")
        << "Az=" << evaluation.azimuthDegrees
        << " El=" << evaluation.elevationDegrees
        << " Roll=" << evaluation.rollDegrees
        << " FoV=" << evaluation.fovDegrees
        << " matches=" << evaluation.matchCount
        << " RMS=" << evaluation.rmsErrorPixels
        << " BrightnessErr=" << evaluation.brightnessRankError
        << " MeanMag=" << evaluation.meanCatalogMagnitude
        << " GuidedScore=" << guidedDirectionEvaluationScore(evaluation)
        << " Cx=" << evaluation.centerOffsetXPixels
        << " Cy=" << evaluation.centerOffsetYPixels
        << " K1=" << evaluation.distortionK1;
}

void CameraPlateSolver::SolverContext::logWeakModePoolDecision(const char *stage, const char *decision, const Evaluation& candidate, double poolQualityRadius, const Evaluation *other)
{
    if (!kLogWeakModeCandidatePools || !candidate.valid) {
        return;
    }
    if (!kLogWeakModeTailRejects && (qstrcmp(decision, "reject-below-tail") == 0)) {
        return;
    }

    qCDebug(cameraPlateSolverLog).noquote().nospace()
        << "CameraPlateSolver[" << stage << "] "
        << decision
        << " score=" << weakModeEvaluationScore(candidate)
        << " matches=" << candidate.matchCount
        << " RMS=" << candidate.rmsErrorPixels
        << " Median=" << medianDistancePixels(candidate.matches)
        << " BrightnessErr=" << candidate.brightnessRankError
        << " MeanMag=" << candidate.meanCatalogMagnitude
        << " poolRmsCap=" << poolQualityRadius
        << " Az=" << candidate.azimuthDegrees
        << " El=" << candidate.elevationDegrees
        << " Roll=" << candidate.rollDegrees
        << " FoV=" << candidate.fovDegrees
        << " K1=" << candidate.distortionK1;

    if (other && other->valid) {
        qCDebug(cameraPlateSolverLog).noquote().nospace()
            << "CameraPlateSolver[" << stage << "] "
            << "compared-to"
            << " score=" << weakModeEvaluationScore(*other)
            << " matches=" << other->matchCount
            << " RMS=" << other->rmsErrorPixels
            << " Median=" << medianDistancePixels(other->matches)
            << " MeanMag=" << other->meanCatalogMagnitude
            << " Az=" << other->azimuthDegrees
            << " El=" << other->elevationDegrees
            << " Roll=" << other->rollDegrees
            << " FoV=" << other->fovDegrees
            << " K1=" << other->distortionK1;
    }
}

void CameraPlateSolver::SolverContext::logWeakModeCandidatePool(const char *stage, const QVector<Evaluation>& candidates)
{
    if (!kLogWeakModeCandidatePools) {
        return;
    }

    qCDebug(cameraPlateSolverLog).noquote().nospace()
        << "CameraPlateSolver[" << stage << "] pool-size=" << candidates.size();
    for (int i = 0; i < candidates.size(); ++i)
    {
        const Evaluation& candidate = candidates.at(i);
        qCDebug(cameraPlateSolverLog).noquote().nospace()
            << "CameraPlateSolver[" << stage << "] #"<< i
            << " score=" << weakModeEvaluationScore(candidate)
            << " matches=" << candidate.matchCount
            << " RMS=" << candidate.rmsErrorPixels
            << " Median=" << medianDistancePixels(candidate.matches)
            << " BrightnessErr=" << candidate.brightnessRankError
            << " MeanMag=" << candidate.meanCatalogMagnitude
            << " Az=" << candidate.azimuthDegrees
            << " El=" << candidate.elevationDegrees
            << " Roll=" << candidate.rollDegrees
            << " FoV=" << candidate.fovDegrees
            << " Cx=" << candidate.centerOffsetXPixels
            << " Cy=" << candidate.centerOffsetYPixels
            << " K1=" << candidate.distortionK1;
    }
}

CameraPlateSolver::SolverContext::Evaluation CameraPlateSolver::SolverContext::searchBestPose(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, QVector<Evaluation>* candidatePool)
{
    Evaluation best;
    if (isCancellationRequested()) {
        return best;
    }
    const bool profilePlateSolve = qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_PROFILE");
    QElapsedTimer searchProfileTimer;
    searchProfileTimer.start();
    auto logSearchProfile = [&](const char *stage, qint64 startedMs) {
        const qint64 elapsedMs = searchProfileTimer.elapsed() - startedMs;
        recordProfileTiming(QStringLiteral("search.%1").arg(QString::fromUtf8(stage)), elapsedMs);
        if (!profilePlateSolve) {
            return;
        }
        qCDebug(cameraPlateSolverLog).noquote().nospace()
            << "CameraPlateSolverProfile search." << stage
            << " elapsedMs=" << elapsedMs
            << " totalMs=" << searchProfileTimer.elapsed()
            << " bestValid=" << best.valid
            << " bestMatches=" << best.matchCount
            << " bestRms=" << best.rmsErrorPixels
            << " pool=" << (candidatePool ? candidatePool->size() : 0);
    };
    const int minMatchCount = std::max(1, settings.m_plateSolveMinMatches);
    const bool useStartFov = plateSolveStartUsesFov(settings);
    const bool useStartElevation = plateSolveStartUsesElevation(settings);
    const bool useStartDirection = plateSolveStartUsesDirection(settings);
    const bool useStartRoll = plateSolveStartUsesRoll(settings);
    const bool useElevationSeedOnly = useStartElevation && !useStartDirection;
    const bool useStartLens = plateSolveStartUsesLens(settings);
    const bool useWeakModeScoring = !useStartDirection;
    const bool useGuidedDirectionScoring = useStartDirection;
    const bool keepMultipleCandidates = candidatePool != nullptr;
    const bool wideWeakMode = useWeakModeScoring && isWidePlateSolveContext(settings);
    const bool useWideFovBlindSeedFirst = wideWeakMode
        && useStartFov
        && !useStartElevation
        && !useStartDirection;
    const bool skipNarrowFovOnlyFullSkyGrids = useStartFov
        && !useStartElevation
        && !useStartDirection
        && !isWidePlateSolveContext(settings)
        && (settings.m_fov < 15.0);
    const int maxMultiHypothesisCandidates = (useStartDirection && (isNarrowField(settings)))
        ? (useStartRoll ? 24 : 256)
        : wideWeakMode ? 64
        : 10;
    const int interestingWeakModeMatchCount = std::max(3, minMatchCount - 1);
    const int weakModeCandidatePoolMinMatches = std::max(3, minMatchCount - 2);
    const double fixedCenterOffsetX = useStartLens ? settings.m_lensCenterOffsetX : 0.0;
    const double fixedCenterOffsetY = useStartLens ? settings.m_lensCenterOffsetY : 0.0;
    const double fixedDistortionK1 = useStartLens ? settings.m_lensDistortionK1 : 0.0;

    const double finalMatchRadius = std::max(1.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius));
    const double maxImageDimension = std::max(imageSize.width(), imageSize.height());
    // WS0 coarse-search tuning knobs, grouped and named so the tuning surface is discoverable and
    // the wide-seed image fraction cannot drift between its (formerly duplicated) FoV/blind uses.
    constexpr double kWideSeedRadiusImageFraction = 0.065;
    constexpr double kWideSeedRadiusMinPixels = 120.0;
    constexpr double kWideSeedRadiusMaxPixels = 240.0;
    constexpr double kGuidedSeedRadiusMatchScale = 4.0;
    constexpr double kGuidedSeedRadiusMaxPixels = 96.0;
    constexpr double kCoarseRollRadiusFovFraction = 0.20;
    constexpr double kCoarseFovRadiusFovFraction = 0.10;
    constexpr double kMinFovRefineStepFovFraction = 0.02;
    constexpr double kFovGridStepFovFraction = 0.5;
    const bool useWideFovSeedRadius = useStartFov
        && !useStartElevation
        && !useStartDirection
        && (settings.m_fov >= 120.0)
        && (settings.m_lensProjection != CameraSettings::LensProjectionRectilinear);
    const bool useWideBlindSeedRadius = !useStartFov
        && !useStartElevation
        && !useStartDirection
        && isWidePlateSolveContext(settings);
    const double wideSeedMatchRadius = std::max(finalMatchRadius,
        std::min(kWideSeedRadiusMaxPixels,
            std::max(kWideSeedRadiusMinPixels, maxImageDimension * kWideSeedRadiusImageFraction)));
    const double wideFovSeedMatchRadius = useWideFovSeedRadius ? wideSeedMatchRadius : finalMatchRadius;
    const double wideBlindSeedMatchRadius = useWideBlindSeedRadius ? wideSeedMatchRadius : finalMatchRadius;
    const double guidedSeedMatchRadius = (useStartDirection && (isNarrowField(settings)))
        ? std::max(finalMatchRadius,
            std::min(kGuidedSeedRadiusMaxPixels, std::max(finalMatchRadius, static_cast<double>(settings.m_plateSolveMatchRadius)) * kGuidedSeedRadiusMatchScale))
        : finalMatchRadius;
    const double searchMatchRadiusOverride = useStartDirection
        ? guidedSeedMatchRadius
        : useWideFovSeedRadius ? wideFovSeedMatchRadius
        : -1.0;
    if (useStartDirection && (isNarrowField(settings))) {
        m_weakModeNormalizationPixels = std::max(m_weakModeNormalizationPixels, guidedSeedMatchRadius);
    }
    if (useWideFovSeedRadius) {
        m_weakModeNormalizationPixels = std::max(m_weakModeNormalizationPixels, wideFovSeedMatchRadius);
    }
    if (useWideBlindSeedRadius) {
        m_weakModeNormalizationPixels = std::max(m_weakModeNormalizationPixels, wideBlindSeedMatchRadius);
    }
    const double coarseSearchRadius = std::max(0.0, settings.m_plateSolveAzElSearchRadius);
    const double coarseRollRadius = useStartRoll
        ? std::max(15.0, std::min(45.0, static_cast<double>(settings.m_fov) * kCoarseRollRadiusFovFraction))
        : std::max(4.0, std::min(20.0, static_cast<double>(settings.m_fov) * kCoarseRollRadiusFovFraction));
    const double coarseFovRadius = std::max(0.05, std::min(12.0, static_cast<double>(settings.m_fov) * kCoarseFovRadiusFovFraction));
    const double minimumFovRefineStep = std::max(0.02, std::min(0.5, static_cast<double>(settings.m_fov) * kMinFovRefineStepFovFraction));

    const double minAzimuthDegrees = 0.0;
    const double maxAzimuthDegrees = 360.0;
    const double azimuthStepDegrees = 5.0;
    const double minElevationDegrees = 0.0;
    const double maxElevationDegrees = 90.0;
    const double elevationStepDegrees = 15.0;
    const double fovGridStepDegrees = std::max(0.25, std::min(5.0, static_cast<double>(settings.m_fov) * kFovGridStepFovFraction));
    const double fovGridAzimuthStepDegrees = fovGridStepDegrees;
    const double fovGridElevationStepDegrees = std::max(0.25, std::min(15.0, static_cast<double>(settings.m_fov) * kFovGridStepFovFraction));

    const std::array<double, 3> coarseFovOffsets = {{-1.0, 0.0, 1.0}};
    const std::array<double, 5> coarseOffsetsOrdered = {{0.0, -0.5, 0.5, -1.0, 1.0}};
    const std::array<double, 3> coarseFovOffsetsOrdered = {{0.0, -1.0, 1.0}};
    // -180° and +180° are numerically identical rotations; keep only -180° to avoid
    // duplicating ~7.7% of blind-grid evaluations.
    const std::array<double, 12> wideRollOffsets = {{-180.0, -150.0, -120.0, -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0, 120.0, 150.0}};
    const std::array<double, 12> wideRollOffsetsOrdered = {{0.0, -30.0, 30.0, -60.0, 60.0, -90.0, 90.0, -120.0, 120.0, -150.0, 150.0, -180.0}};
    const std::array<double, 24> wideRollOffsetsFineOrdered = {{
        0.0, -15.0, 15.0, -30.0, 30.0, -45.0, 45.0, -60.0, 60.0,
        -75.0, 75.0, -90.0, 90.0, -105.0, 105.0, -120.0, 120.0,
        -135.0, 135.0, -150.0, 150.0, -165.0, 165.0, -180.0
    }};
    QVector<int> narrowGuidedFirstPassCatalogIndices;
    const QVector<int>* guidedFirstPassCatalogIndices = nullptr;
    if (useStartDirection && (isNarrowField(settings)))
    {
        const double localRadiusDegrees = std::max(
            static_cast<double>(settings.m_plateSolveAzElSearchRadius) + static_cast<double>(settings.m_fov) * 2.0,
            static_cast<double>(settings.m_fov) * 4.0);
        const QVector<VisibleCatalogStar> localBrightStars = selectLocalVisibleStars(
            catalogContext.visibleStars,
            settings.m_azimuth,
            settings.m_elevation,
            localRadiusDegrees,
            256);
        const double firstPassMaxMagnitude = std::min(
            static_cast<double>(settings.m_plateSolveMaxMagnitude),
            kNarrowGuidedBrightCatalogMaxMagnitude);
        narrowGuidedFirstPassCatalogIndices.reserve(localBrightStars.size());
        for (const VisibleCatalogStar& star : localBrightStars)
        {
            if (star.magnitude > firstPassMaxMagnitude) {
                break;
            }
            narrowGuidedFirstPassCatalogIndices.append(star.catalogIndex);
        }
        if (narrowGuidedFirstPassCatalogIndices.size() >= std::max(2, minMatchCount - 1)) {
            guidedFirstPassCatalogIndices = &narrowGuidedFirstPassCatalogIndices;
        }
    }

    QVector<double> fovSearchRollOffsets;
    if (wideWeakMode && useStartFov && !useStartElevation && !useStartDirection) {
        fovSearchRollOffsets.reserve(static_cast<int>(wideRollOffsetsFineOrdered.size()));
        for (double rollOffset : wideRollOffsetsFineOrdered) {
            fovSearchRollOffsets.append(rollOffset);
        }
    } else {
        fovSearchRollOffsets.reserve(static_cast<int>(wideRollOffsetsOrdered.size()));
        for (double rollOffset : wideRollOffsetsOrdered) {
            fovSearchRollOffsets.append(rollOffset);
        }
    }
    QVector<int> wideFirstPassCatalogIndices;
    if (wideWeakMode && useStartFov)
    {
        wideFirstPassCatalogIndices.reserve(std::min(192, static_cast<int>(catalogContext.visibleStars.size())));
        for (const VisibleCatalogStar& star : catalogContext.visibleStars)
        {
            if ((star.magnitude <= kWideFovBrightFirstPassMaxMagnitude)
                || (wideFirstPassCatalogIndices.size() < 96))
            {
                wideFirstPassCatalogIndices.append(star.catalogIndex);
            }
            if (wideFirstPassCatalogIndices.size() >= 192) {
                break;
            }
        }
    }

    auto evaluateSeed = [&](const char *stage,
                            double azimuthDegrees,
                            double elevationDegrees,
                            double rollDegrees,
                            double fovDegrees,
                            double matchRadiusOverride = -1.0,
                            const QVector<int>* allowedCatalogIndices = nullptr) {
        if (isCancellationRequested()) {
            return;
        }
        const Evaluation candidate = evaluatePose(
            settings,
            catalogContext,
            imageSize,
            captureDateTimeUtc,
            starDetections,
            detectionIndices,
            azimuthDegrees,
            elevationDegrees,
            rollDegrees,
            fovDegrees,
            allowedCatalogIndices,
            fixedCenterOffsetX,
            fixedCenterOffsetY,
            fixedDistortionK1,
            matchRadiusOverride);
        logPlateSolveEvaluation(stage, candidate);
        if (keepMultipleCandidates) {
            insertDistinctEvaluationCandidate(
                *candidatePool,
                candidate,
                maxMultiHypothesisCandidates,
                useWeakModeScoring,
                stage,
                interestingWeakModeMatchCount,
                weakModeCandidatePoolMinMatches,
                useGuidedDirectionScoring);
        }
        if (isBetterEvaluationForMode(candidate, best, useWeakModeScoring, useGuidedDirectionScoring)) {
            best = candidate;
            logPlateSolveEvaluation(stage, best, true);
        }
    };

    // Lightweight variant used by the blind-grid roll sweep.  The projected
    // catalog is already stored in m_blindGridProjectedScratch — no
    // createProjector or buildProjectedCatalogInto needed.
    auto evaluateSeedFromCache = [&](const char *stage,
                                     double azimuthDegrees,
                                     double elevationDegrees,
                                     double rollDegrees,
                                     double fovDegrees,
                                     double matchRadiusPixels,
                                     bool promoteSparseGuidedPair = true) {
        if (isCancellationRequested()) {
            return;
        }
        const Evaluation candidate = evaluatePoseFromPrecomputedCatalog(
            settings, catalogContext, starDetections, detectionIndices,
            azimuthDegrees, elevationDegrees, rollDegrees, fovDegrees,
            fixedCenterOffsetX, fixedCenterOffsetY, fixedDistortionK1,
            matchRadiusPixels);
        logPlateSolveEvaluation(stage, candidate);
        if (keepMultipleCandidates) {
            if (promoteSparseGuidedPair)
            {
                const Evaluation promotedSparseGuidedPair = promoteSparseGuidedPairFromMatches(
                    settings,
                    catalogContext,
                    starDetections,
                    candidate);
                if (promotedSparseGuidedPair.valid)
                {
                    const bool alreadyQueued = std::any_of(candidatePool->cbegin(), candidatePool->cend(), [&promotedSparseGuidedPair](const Evaluation& existing) {
                        return sameEvaluationIdentity(existing, promotedSparseGuidedPair);
                    });
                    if (!alreadyQueued) {
                        candidatePool->append(promotedSparseGuidedPair);
                    }
                }
            }
            insertDistinctEvaluationCandidate(
                *candidatePool, candidate, maxMultiHypothesisCandidates,
                useWeakModeScoring, stage, interestingWeakModeMatchCount,
                weakModeCandidatePoolMinMatches, useGuidedDirectionScoring);
        }
        if (isBetterEvaluationForMode(candidate, best, useWeakModeScoring, useGuidedDirectionScoring)) {
            best = candidate;
            logPlateSolveEvaluation(stage, best, true);
        }
    };

    auto hasGoodGuidedSeed = [&]() {
        if (useStartDirection) {
            if (!isAcceptableDirectionSeedSolve(settings, minMatchCount, best)) {
                return false;
            }
            if (isWidePlateSolveContext(settings)) {
                return true;
            }
            return hasAcceptableBrightnessConsistency(best);
        }
        if (useElevationSeedOnly) {
            return isAcceptableElevationSeedEvaluation(settings, minMatchCount, best);
        }
        if (useStartFov) {
            return isStrongGuidedSolve(settings, minMatchCount, best);
        }
        return false;
    };

    auto appendUniqueOffset = [](QVector<double>& offsets, double value) {
        for (double existing : offsets)
        {
            if (std::fabs(existing - value) < 1e-6) {
                return;
            }
        }
        offsets.append(value);
    };

    auto buildGuidedDirectionOffsets = [&](double radius) {
        QVector<double> offsets;
        appendUniqueOffset(offsets, 0.0);
        if (radius <= 0.0) {
            return offsets;
        }

        const double fineStep = std::min(radius, std::max(0.02, static_cast<double>(settings.m_fov) * 0.04));
        const double fovStep = std::min(radius, std::max(0.10, static_cast<double>(settings.m_fov) * 0.45));
        const double fovDegrees = std::max(0.01, static_cast<double>(settings.m_fov));
        for (double value : {
            fineStep,
            fineStep * 2.0,
            fineStep * 4.0,
            fovStep,
            fovStep * 2.0,
            fovDegrees * 0.5,
            fovDegrees * 0.75,
            fovDegrees,
            fovDegrees * 1.25,
            fovDegrees * 1.5,
            fovDegrees * 2.0,
            radius * 0.5,
            radius})
        {
            const double clamped = std::min(radius, value);
            appendUniqueOffset(offsets, -clamped);
            appendUniqueOffset(offsets, clamped);
        }
        return offsets;
    };

    auto buildGuidedRollOffsets = [&]() {
        QVector<double> offsets;
        if (useStartRoll)
        {
            for (double factor : {0.0, -0.5, 0.5, -1.0, 1.0}) {
                appendUniqueOffset(offsets, factor * coarseRollRadius);
            }
            return offsets;
        }

        const double step = (isNarrowField(settings)) ? 10.0
            : (settings.m_fov <= 15.0) ? 15.0
            : 30.0;
        appendUniqueOffset(offsets, 0.0);
        for (double value = step; value <= 180.0 + 1e-6; value += step)
        {
            appendUniqueOffset(offsets, -std::min(180.0, value));
            appendUniqueOffset(offsets, std::min(180.0, value));
        }
        return offsets;
    };

    if (useStartDirection)
    {
        const qint64 stageStartMs = searchProfileTimer.elapsed();
        bool guidedSatisfied = false;
        const bool allowGuidedEarlyStop = (settings.m_fov < kWideFovMagnitudePreferenceThresholdDegrees)
            && useStartRoll;
        const QVector<double> directionOffsets = buildGuidedDirectionOffsets(coarseSearchRadius);
        const QVector<double> rollOffsets = buildGuidedRollOffsets();

        // T2 (P4): the no-early-stop guided grid (mode-3 narrow etc., the dominant searchBestPose
        // cost) is a FIXED set of evaluations — guidedSatisfied can never fire when
        // !allowGuidedEarlyStop, so the loop has no data-dependent exits. That makes it safe to
        // split into (a) a parallel, per-worker-context evaluation of every (fov,el,az,roll) grid
        // point (each worker owns its blind-grid cache/scratch; evaluatePoseFromPrecomputedCatalog
        // reads only that and the shared read-only inputs), then (b) a SERIAL replay of the
        // reduction — logging, sparse-pair promotion, candidate-pool insertion, best-tracking — in
        // the exact canonical fov→el→az→roll order of the serial loop, on this context. The replay
        // is the same code path evaluateSeedFromCache runs serially, so the result is
        // byte-identical by construction (only under mid-run cancellation can the set of completed
        // evaluations differ, and a cancelled solve's result is discarded anyway). The early-stop
        // mode keeps the original serial loop: its exits are data-dependent and it is cheap.
        struct GuidedGridPoint
        {
            double azimuthDegrees = 0.0;
            double elevationDegrees = 0.0;
            double rollDegrees = 0.0;
            double fovDegrees = 0.0;
            int cellIndex = -1;
        };
        QVector<GuidedGridPoint> gridPoints;
        if (!allowGuidedEarlyStop)
        {
            gridPoints.reserve(static_cast<qsizetype>(coarseFovOffsetsOrdered.size())
                * directionOffsets.size() * directionOffsets.size() * rollOffsets.size());
            int cellIndex = 0;
            for (double fovFactor : coarseFovOffsetsOrdered)
            {
                for (double elOffset : directionOffsets)
                {
                    for (double azOffset : directionOffsets)
                    {
                        const double fovDegrees = std::max(
                            static_cast<double>(CameraSettings::m_minFov),
                            static_cast<double>(settings.m_fov) + fovFactor * coarseFovRadius);
                        for (double rollOffset : rollOffsets)
                        {
                            GuidedGridPoint point;
                            point.azimuthDegrees = settings.m_azimuth + azOffset;
                            point.elevationDegrees = settings.m_elevation + elOffset;
                            point.rollDegrees = settings.m_roll + rollOffset;
                            point.fovDegrees = fovDegrees;
                            point.cellIndex = cellIndex;
                            gridPoints.append(point);
                        }
                        ++cellIndex;
                    }
                }
            }
        }
        const int guidedGridThreads = gridPoints.isEmpty() ? 1
            : refinementWorkerThreadCount(
                static_cast<int>(gridPoints.size()),
                estimateRefinementWorkUnits(
                    static_cast<int>(gridPoints.size()),
                    catalogContext.visibleStars.size(),
                    detectionIndices.size()));

        if (!allowGuidedEarlyStop && (guidedGridThreads > 1))
        {
            const int pointCount = static_cast<int>(gridPoints.size());
            const int rollCount = std::max(1, static_cast<int>(rollOffsets.size()));
            QVector<Evaluation> gridResults(pointCount);
            QThreadPool guidedGridPool;
            guidedGridPool.setMaxThreadCount(guidedGridThreads);
            for (int workerIndex = 0; workerIndex < guidedGridThreads; ++workerIndex)
            {
                guidedGridPool.start(QRunnable::create([&, workerIndex]() {
                    SolverContext workerContext(m_owner);
                    workerContext.copySearchStateFrom(*this);
                    int cachedCellIndex = -1;
                    // Stride by CELL so each worker builds a cell's blind-grid cache once and
                    // reuses it across that cell's rolls, mirroring the serial loop's cost shape.
                    const int cellCount = (pointCount + rollCount - 1) / rollCount;
                    for (int cell = workerIndex; cell < cellCount; cell += guidedGridThreads)
                    {
                        if (workerContext.isCancellationRequested()) {
                            break;
                        }
                        const int firstPoint = cell * rollCount;
                        const int lastPoint = std::min(pointCount, firstPoint + rollCount);
                        const GuidedGridPoint& cellPoint = gridPoints[firstPoint];
                        const SkyProjector refProjector = createProjector(
                            settings,
                            imageSize,
                            cellPoint.azimuthDegrees,
                            cellPoint.elevationDegrees,
                            0.0,
                            cellPoint.fovDegrees,
                            fixedCenterOffsetX,
                            fixedCenterOffsetY,
                            fixedDistortionK1);
                        if (cachedCellIndex != cellPoint.cellIndex)
                        {
                            workerContext.buildBlindGridCache(
                                catalogContext,
                                refProjector,
                                guidedFirstPassCatalogIndices);
                            cachedCellIndex = cellPoint.cellIndex;
                        }
                        for (int pointIndex = firstPoint; pointIndex < lastPoint; ++pointIndex)
                        {
                            if (workerContext.isCancellationRequested()) {
                                break;
                            }
                            const GuidedGridPoint& point = gridPoints[pointIndex];
                            workerContext.populateBlindGridProjectedCatalog(
                                point.rollDegrees, guidedSeedMatchRadius, refProjector);
                            gridResults[pointIndex] = workerContext.evaluatePoseFromPrecomputedCatalog(
                                settings, catalogContext, starDetections, detectionIndices,
                                point.azimuthDegrees, point.elevationDegrees, point.rollDegrees,
                                point.fovDegrees,
                                fixedCenterOffsetX, fixedCenterOffsetY, fixedDistortionK1,
                                guidedSeedMatchRadius);
                        }
                    }
                }));
            }
            guidedGridPool.waitForDone();

            // Serial replay of the reduction in canonical order — identical code path to the
            // serial evaluateSeedFromCache (promoteSparseGuidedPair disabled there too).
            for (int pointIndex = 0; pointIndex < pointCount; ++pointIndex)
            {
                if (isCancellationRequested()) {
                    break;
                }
                const Evaluation& candidate = gridResults[pointIndex];
                logPlateSolveEvaluation("guided-direction", candidate);
                if (keepMultipleCandidates) {
                    insertDistinctEvaluationCandidate(
                        *candidatePool, candidate, maxMultiHypothesisCandidates,
                        useWeakModeScoring, "guided-direction", interestingWeakModeMatchCount,
                        weakModeCandidatePoolMinMatches, useGuidedDirectionScoring);
                }
                if (isBetterEvaluationForMode(candidate, best, useWeakModeScoring, useGuidedDirectionScoring)) {
                    best = candidate;
                    logPlateSolveEvaluation("guided-direction", best, true);
                }
            }
        }
        else
        {
        for (double fovFactor : coarseFovOffsetsOrdered)
        {
            if (isCancellationRequested()) break;
            if (guidedSatisfied) break;
            for (double elOffset : directionOffsets)
            {
                if (isCancellationRequested()) break;
                if (guidedSatisfied) break;
                for (double azOffset : directionOffsets)
                {
                    if (isCancellationRequested()) break;
                    if (guidedSatisfied) break;
                    const double azimuthDegrees = settings.m_azimuth + azOffset;
                    const double elevationDegrees = settings.m_elevation + elOffset;
                    const double fovDegrees = std::max(
                        static_cast<double>(CameraSettings::m_minFov),
                        static_cast<double>(settings.m_fov) + fovFactor * coarseFovRadius);
                    const SkyProjector refProjector = createProjector(
                        settings,
                        imageSize,
                        azimuthDegrees,
                        elevationDegrees,
                        0.0,
                        fovDegrees,
                        fixedCenterOffsetX,
                        fixedCenterOffsetY,
                        fixedDistortionK1);
                    buildBlindGridCache(
                        catalogContext,
                        refProjector,
                        guidedFirstPassCatalogIndices);
                    for (double rollOffset : rollOffsets)
                    {
                        if (isCancellationRequested()) break;
                        const double rollDegrees = settings.m_roll + rollOffset;
                        populateBlindGridProjectedCatalog(rollDegrees, guidedSeedMatchRadius, refProjector);
                        evaluateSeedFromCache(
                            "guided-direction",
                            azimuthDegrees,
                            elevationDegrees,
                            rollDegrees,
                            fovDegrees,
                            guidedSeedMatchRadius,
                            false);
                        if (allowGuidedEarlyStop && hasGoodGuidedSeed()) {
                            guidedSatisfied = true;
                            break;
                        }
                    }
                }
            }
        }
        }
        logSearchProfile("guided-direction", stageStartMs);
    }
    else if (useStartElevation)
    {
        const qint64 stageStartMs = searchProfileTimer.elapsed();
        const std::array<double, 5> elevationSeedFovScales = {{1.00, 0.85, 1.15, 0.70, 1.30}};
        for (double fovScale : elevationSeedFovScales)
        {
            if (isCancellationRequested()) break;
            for (double elFactor : coarseOffsetsOrdered)
            {
                if (isCancellationRequested()) break;
                for (double azimuthDegrees = minAzimuthDegrees; azimuthDegrees < maxAzimuthDegrees; azimuthDegrees += fovGridAzimuthStepDegrees)
                {
                    if (isCancellationRequested()) break;
                    for (double rollDegrees : fovSearchRollOffsets)
                    {
                        if (isCancellationRequested()) break;
                        evaluateSeed(
                            "guided-elevation",
                            azimuthDegrees,
                            settings.m_elevation + elFactor * coarseSearchRadius,
                            rollDegrees,
                            std::clamp(static_cast<double>(settings.m_fov) * fovScale,
                                static_cast<double>(CameraSettings::m_minFov),
                                static_cast<double>(CameraSettings::m_maxFov)));
                    }
                }
            }
        }

        if (best.valid && (best.matchCount >= std::max(2, minMatchCount - 1)))
        {
            const std::array<double, 5> azimuthOffsets = {{-2.0, -1.0, 0.0, 1.0, 2.0}};
            const std::array<double, 3> elevationOffsets = {{-1.0, 0.0, 1.0}};
            const std::array<double, 5> rollOffsets = {{-2.0, -1.0, 0.0, 1.0, 2.0}};
            const std::array<double, 3> refineFovScales = {{0.92, 1.00, 1.08}};
            const double azimuthRefineStep = fovGridAzimuthStepDegrees;
            const double elevationRefineStep = std::max(0.25, std::min(fovGridElevationStepDegrees, coarseSearchRadius * 0.25));
            const double rollRefineStep = std::max(0.25, std::min(5.0, static_cast<double>(settings.m_fov) * 0.5));

            for (double azimuthOffset : azimuthOffsets)
            {
                if (isCancellationRequested()) break;
                for (double elevationOffset : elevationOffsets)
                {
                    if (isCancellationRequested()) break;
                    for (double rollOffset : rollOffsets)
                    {
                        if (isCancellationRequested()) break;
                        for (double fovScale : refineFovScales)
                        {
                            if (isCancellationRequested()) break;
                            evaluateSeed(
                                "guided-elevation-refine",
                                best.azimuthDegrees + azimuthOffset * azimuthRefineStep,
                                best.elevationDegrees + elevationOffset * elevationRefineStep,
                                best.rollDegrees + rollOffset * rollRefineStep,
                                std::clamp(best.fovDegrees * fovScale,
                                    static_cast<double>(CameraSettings::m_minFov),
                                    static_cast<double>(CameraSettings::m_maxFov)));
                        }
                    }
                }
            }
        }
        logSearchProfile("guided-elevation", stageStartMs);
    }
    else if (useStartFov)
    {
        const qint64 stageStartMs = searchProfileTimer.elapsed();
        const double guidedFovMatchRadius = useWideFovSeedRadius
            ? wideFovSeedMatchRadius
            : static_cast<double>(settings.m_plateSolveMatchRadius);
        const bool runGuidedFovGrid = !useWideFovBlindSeedFirst
            && !skipNarrowFovOnlyFullSkyGrids;
        if (runGuidedFovGrid)
        {
            for (double fovFactor : coarseFovOffsetsOrdered)
            {
                if (isCancellationRequested()) break;
                for (double elevationDegrees = minElevationDegrees; elevationDegrees <= maxElevationDegrees; elevationDegrees += fovGridElevationStepDegrees)
                {
                    if (isCancellationRequested()) break;
                    for (double azimuthDegrees = minAzimuthDegrees; azimuthDegrees < maxAzimuthDegrees; azimuthDegrees += fovGridAzimuthStepDegrees)
                    {
                        if (isCancellationRequested()) break;
                        if (wideWeakMode)
                        {
                            const double fovDegrees = std::max(
                                static_cast<double>(CameraSettings::m_minFov),
                                static_cast<double>(settings.m_fov) + fovFactor * coarseFovRadius);
                            const SkyProjector refProjector = createProjector(
                                settings,
                                imageSize,
                                azimuthDegrees,
                                elevationDegrees,
                                0.0,
                                fovDegrees,
                                fixedCenterOffsetX,
                                fixedCenterOffsetY,
                                fixedDistortionK1);
                            buildBlindGridCache(
                                catalogContext,
                                refProjector,
                                wideFirstPassCatalogIndices.isEmpty() ? nullptr : &wideFirstPassCatalogIndices);
                            // Use fovSearchRollOffsets, which in this branch (wideWeakMode &&
                            // useStartFov && !useStartElevation && !useStartDirection) is the fine
                            // 15-deg roll set. It was built for exactly this mode-1 path but was
                            // previously only referenced from the useStartElevation branch (where it
                            // resolves to the coarse set), so the fine sweep was never actually used.
                            // The finer roll granularity closes a mode-1 wide/fisheye coverage gap.
                            for (double rollDegrees : fovSearchRollOffsets)
                            {
                                if (isCancellationRequested()) break;
                                populateBlindGridProjectedCatalog(rollDegrees, guidedFovMatchRadius, refProjector);
                                evaluateSeedFromCache(
                                    "guided-fov",
                                    azimuthDegrees,
                                    elevationDegrees,
                                    rollDegrees,
                                    fovDegrees,
                                    guidedFovMatchRadius);
                            }
                        }
                        else
                        {
                            for (double rollDegrees : wideRollOffsetsOrdered)
                            {
                                if (isCancellationRequested()) break;
                                evaluateSeed(
                                    "guided-fov",
                                    azimuthDegrees,
                                    elevationDegrees,
                                    rollDegrees,
                                    std::max(static_cast<double>(CameraSettings::m_minFov),
                                             static_cast<double>(settings.m_fov) + fovFactor * coarseFovRadius),
                                    useWideFovSeedRadius ? wideFovSeedMatchRadius : -1.0,
                                    wideFirstPassCatalogIndices.isEmpty() ? nullptr : &wideFirstPassCatalogIndices);
                            }
                        }
                    }
                }
            }
        }

        logSearchProfile("guided-fov", stageStartMs);
    }

    const bool needBlindSearch = !useStartFov
        || (useStartDirection
            ? !hasGoodGuidedSeed()
            : useElevationSeedOnly
                ? !isAcceptableElevationSeedEvaluation(settings, minMatchCount, best)
            : !isStrongGuidedSolve(settings, minMatchCount, best));

    if (needBlindSearch)
    {
        const qint64 stageStartMs = searchProfileTimer.elapsed();
        QVector<VisibleCatalogStar> localVisibleStars;
        const QVector<VisibleCatalogStar>* blindVisibleStars = &catalogContext.visibleStars;
        if (useStartDirection && (isNarrowField(settings)))
        {
            const double localRadiusDegrees = std::max(
                static_cast<double>(settings.m_plateSolveAzElSearchRadius) + static_cast<double>(settings.m_fov) * 2.0,
                static_cast<double>(settings.m_fov) * 4.0);
            localVisibleStars = selectLocalVisibleStars(
                catalogContext.visibleStars,
                settings.m_azimuth,
                settings.m_elevation,
                localRadiusDegrees,
                2048);
            if (!localVisibleStars.isEmpty()) {
                blindVisibleStars = &localVisibleStars;
            }
            qCDebug(cameraPlateSolverLog) << "CameraPlateSolver: guided narrow blind seed catalog"
                     << "stars" << blindVisibleStars->size()
                     << "radius" << localRadiusDegrees;
        }
        else if (useElevationSeedOnly)
        {
            const double localElevationRadiusDegrees = std::max(
                static_cast<double>(settings.m_plateSolveAzElSearchRadius) + static_cast<double>(settings.m_fov) * 0.5,
                static_cast<double>(settings.m_fov));
            localVisibleStars = selectVisibleStarsNearElevation(
                catalogContext.visibleStars,
                settings.m_elevation,
                localElevationRadiusDegrees,
                128);
            if (!localVisibleStars.isEmpty()) {
                blindVisibleStars = &localVisibleStars;
            }
        }

        auto consumeBlindSeeds = [&](const QVector<Evaluation>& seeds, const char *stage) {
            for (const Evaluation& seed : seeds)
            {
                logPlateSolveEvaluation(stage, seed);
                if (candidatePool) {
                    if (seed.sparseGuidedPair)
                    {
                        const bool alreadyQueued = std::any_of(candidatePool->cbegin(), candidatePool->cend(), [&seed](const Evaluation& existing) {
                            return sameEvaluationIdentity(existing, seed);
                        });
                        if (!alreadyQueued) {
                            candidatePool->append(seed);
                        }
                    }
                    else
                    {
                        insertDistinctEvaluationCandidate(
                            *candidatePool,
                            seed,
                            maxMultiHypothesisCandidates,
                            useWeakModeScoring,
                            stage,
                            interestingWeakModeMatchCount,
                            weakModeCandidatePoolMinMatches,
                            useGuidedDirectionScoring);
                    }
                }
                if (isBetterEvaluationForMode(seed, best, useWeakModeScoring, useGuidedDirectionScoring)) {
                    best = seed;
                    logPlateSolveEvaluation(stage, best, true);
                }
            }
        };
        auto isStrongWideWeakBlindSeed = [&](const Evaluation& candidate) {
            const double strongWideSeedRmsCap = std::min(
                std::max(static_cast<double>(settings.m_plateSolveMatchRadius) * 0.9, 2.0) * 0.60,
                14.0);
            return wideWeakMode
                && candidate.valid
                && (candidate.matchCount >= std::max(10, minMatchCount + 6))
                && (candidate.rmsErrorPixels <= strongWideSeedRmsCap);
        };
        auto hasGoodWideBlindSeed = [&]() {
            return wideWeakMode
                && ((isStrongBlindSeedEvaluation(settings, detectionIndices, best)
                        && hasAcceptableBrightnessConsistency(best))
                    || isStrongWideWeakBlindSeed(best));
        };

        // Ablation harness hook: SDRANGEL_CAMERA_PLATE_SOLVER_DISABLE_SEEDS is a comma-separated
        // list of seed-engine tokens (brighttriangle, brightpair) to skip, so each engine's
        // marginal contribution to the suite can be measured. The vector-quad engine has its own
        // SDRANGEL_CAMERA_PLATE_SOLVER_DISABLE_QUAD_INDEX toggle.
        const QString disabledSeedTokens = qEnvironmentVariable("SDRANGEL_CAMERA_PLATE_SOLVER_DISABLE_SEEDS");
        const auto seedDisabled = [&disabledSeedTokens](const char* token) {
            if (disabledSeedTokens.isEmpty()) {
                return false;
            }
            const QStringList tokens = disabledSeedTokens.split(QLatin1Char(','), Qt::SkipEmptyParts);
            for (const QString& t : tokens) {
                if (t.trimmed().compare(QLatin1String(token), Qt::CaseInsensitive) == 0) {
                    return true;
                }
            }
            return false;
        };

        const bool useBrightGuidedTriangles = useStartDirection
            && !wideWeakMode
            && (isNarrowField(settings))
            && !seedDisabled("brighttriangle");

        qint64 seedStageStartMs = searchProfileTimer.elapsed();
        if (isCancellationRequested()) {
            return best;
        }
        QVector<Evaluation> brightTriangleSeeds;
        if (useBrightGuidedTriangles)
        {
            brightTriangleSeeds = buildBrightGuidedAnchorTriangleSeeds(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                *blindVisibleStars);
            logSearchProfile("bright-anchor-triangle-seeds", seedStageStartMs);
            const qint64 ratioStageStartMs = searchProfileTimer.elapsed();
            const bool haveAcceptableAnchorTriangle = std::any_of(
                brightTriangleSeeds.cbegin(),
                brightTriangleSeeds.cend(),
                [&](const Evaluation& seed) {
                    return isAcceptableDirectionSeedSolve(settings, minMatchCount, seed)
                        && hasAcceptableBrightnessConsistency(seed);
                });
            if (!haveAcceptableAnchorTriangle)
            {
                const QVector<Evaluation> ratioTriangleSeeds = buildBrightGuidedTriangleSeeds(
                    settings,
                    catalogContext,
                    imageSize,
                    captureDateTimeUtc,
                    starDetections,
                    detectionIndices,
                    *blindVisibleStars);
                brightTriangleSeeds += ratioTriangleSeeds;
            }
            logSearchProfile("bright-ratio-triangle-seeds", ratioStageStartMs);
        }
        logSearchProfile("bright-triangle-seeds", seedStageStartMs);
        consumeBlindSeeds(brightTriangleSeeds, "bright-triangle-seed");

        const bool brightTriangleSeedAlreadyAcceptable = useBrightGuidedTriangles
            && isAcceptableDirectionSeedSolve(settings, minMatchCount, best)
            && hasAcceptableBrightnessConsistency(best);

        seedStageStartMs = searchProfileTimer.elapsed();
        if (isCancellationRequested()) {
            return best;
        }
        const QVector<Evaluation> brightPairSeeds = (wideWeakMode && !seedDisabled("brightpair"))
            ? buildBrightPairSeeds(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                *blindVisibleStars)
            : QVector<Evaluation>();
        logSearchProfile("bright-pair-seeds", seedStageStartMs);
        const bool consumeBrightPairsBeforeTriangle = wideWeakMode;
        if (consumeBrightPairsBeforeTriangle) {
            consumeBlindSeeds(brightPairSeeds, "bright-pair-seed");
        }

        // For mode-1 (FoV known) wide/weak-mode images, run the vector-quad engine
        // here, before the bright-pair "good enough" gate below can short-circuit
        // the rest of the seed pipeline. brightPairSeeds alone routinely satisfies
        // hasGoodWideBlindSeed() for wide/fisheye images even when the resulting
        // seed doesn't lead to a successful solve (e.g. stars-wide-2.jpg), which
        // made this engine dead code when reached only via the fallback chain below.
        bool vectorQuadSeedsRunEarly = false;
        if (wideWeakMode && plateSolveStartUsesFov(settings)
            && !qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_DISABLE_QUAD_INDEX"))
        {
            seedStageStartMs = searchProfileTimer.elapsed();
            if (isCancellationRequested()) {
                return best;
            }
            const QVector<Evaluation> vectorQuadSeeds = buildVectorQuadBlindSeeds(
                settings,
                catalogContext,
                imageSize,
                captureDateTimeUtc,
                starDetections,
                detectionIndices,
                *blindVisibleStars);
            logSearchProfile("vector-quad-seeds", seedStageStartMs);
            consumeBlindSeeds(vectorQuadSeeds, "vector-quad-seed");
            vectorQuadSeedsRunEarly = true;
        }
        else if (wideWeakMode && !plateSolveStartUsesFov(settings)
            && !qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_DISABLE_QUAD_INDEX"))
        {
            // Mode 0 (fully blind, FoV unknown) on a wide fisheye lens: the
            // vector-quad engine needs an assumed FoV to unproject pixels to rays,
            // so sweep a coarse fisheye-biased set of candidate FoVs and run the
            // bright-pool quad matcher for each. The correct FoV yields matching
            // quad codes (the 5-D code's scale dimension rejects wrong FoVs); a
            // wide fisheye at zenith captures almost the whole visible hemisphere,
            // so the brightest whole-sky catalog stars are essentially all in frame
            // and the bright-pool catalog index covers them. Stop early once a
            // strong seed is found. This is purely additive - the wide-fallback grid
            // stage still runs below.
            seedStageStartMs = searchProfileTimer.elapsed();
            static const double kBlindQuadFovSweepDegrees[] = {180.0, 160.0, 140.0, 120.0, 95.0, 70.0, 45.0};
            for (double candidateFov : kBlindQuadFovSweepDegrees)
            {
                if (isCancellationRequested()) {
                    return best;
                }
                const QVector<Evaluation> vectorQuadSeeds = buildVectorQuadBlindSeeds(
                    settings,
                    catalogContext,
                    imageSize,
                    captureDateTimeUtc,
                    starDetections,
                    detectionIndices,
                    *blindVisibleStars,
                    candidateFov);
                consumeBlindSeeds(vectorQuadSeeds, "vector-quad-seed");
                if (hasGoodWideBlindSeed()) {
                    break;
                }
            }
            logSearchProfile("vector-quad-seeds", seedStageStartMs);
            vectorQuadSeedsRunEarly = true;
        }

        const bool brightPairSeedAlreadyAcceptable = useStartDirection
            && !wideWeakMode
            && isAcceptableDirectionSeedSolve(settings, minMatchCount, best)
            && hasAcceptableBrightnessConsistency(best);
        const bool wideBrightPairSeedAlreadyAcceptable = wideWeakMode && hasGoodWideBlindSeed();
        // The legacy blind-triangle and ratio blind-quad seed engines were removed here. Ablation
        // across the REAL + fisheye + wide + narrow synthetic corpora (test/seed-ablation.ps1)
        // showed blind-quad contributed ZERO unique solves and blind-triangle only one synthetic
        // case; the bright-guided / bright-pair seeds, the vector-quad engine, and the wide-fallback
        // grid below cover their role. (The core buildBlindTriangleSeeds routine is retained -- it
        // is still used as a subroutine by the bright-guided triangle engine.)
        if (!brightTriangleSeedAlreadyAcceptable
            && !brightPairSeedAlreadyAcceptable
            && !wideBrightPairSeedAlreadyAcceptable)
        {
            if (!wideWeakMode && !consumeBrightPairsBeforeTriangle) {
                consumeBlindSeeds(brightPairSeeds, "bright-pair-seed");
            }

            // Vector quad-hash engine (assumed/known FoV). For wideWeakMode mode-1 images it
            // already ran earlier (before the bright-pair short-circuit) -- don't run it twice.
            if (!hasGoodWideBlindSeed()
                && !vectorQuadSeedsRunEarly
                && !qEnvironmentVariableIsSet("SDRANGEL_CAMERA_PLATE_SOLVER_DISABLE_QUAD_INDEX"))
            {
                seedStageStartMs = searchProfileTimer.elapsed();
                if (isCancellationRequested()) {
                    return best;
                }
                const QVector<Evaluation> vectorQuadSeeds = buildVectorQuadBlindSeeds(
                    settings,
                    catalogContext,
                    imageSize,
                    captureDateTimeUtc,
                    starDetections,
                    detectionIndices,
                    *blindVisibleStars);
                logSearchProfile("vector-quad-seeds", seedStageStartMs);
                consumeBlindSeeds(vectorQuadSeeds, "vector-quad-seed");
            }
        }
        logSearchProfile("blind-seeds", stageStartMs);
    }

    // Short-circuit the exhaustive wide-fallback grid when the blind seeds have already
    // landed in a reasonable basin. The grid runs ~52k match evaluations (72 az × 7 el ×
    // 13 roll × 8 fov — but only ~4 k full catalog projections thanks to the roll-sweep
    // cache), so any half-decent prior result is worth keeping. Acceptance bar:
    // at least half the minimum required matches *and* an RMS that fits inside the
    // acquisition radius — the subsequent refinement loops will tighten this further.
    const double wideFallbackRmsCap = std::max(
        static_cast<double>(settings.m_plateSolveMatchRadius) * 0.9,
        2.0);
    const bool blindSeedAlreadyAcceptable = best.valid
        && (best.matchCount >= std::max(2, minMatchCount / 2))
        && (best.rmsErrorPixels <= wideFallbackRmsCap);
    const bool blindSeedVeryStrong = wideWeakMode
        && best.valid
        && (best.matchCount >= std::max(10, minMatchCount + 6))
        && (best.rmsErrorPixels <= std::min(wideFallbackRmsCap * 0.60, 14.0));
    const bool wideWeakBestLooksLikeFalseBrightMatch = wideWeakMode
        && !blindSeedVeryStrong
        && best.valid
        && ((!hasAcceptableBrightnessConsistency(best))
            || (std::isfinite(best.meanCatalogMagnitude) && (best.meanCatalogMagnitude > 3.0)));

    if (!skipNarrowFovOnlyFullSkyGrids
        && (((!best.valid || (best.matchCount < minMatchCount)) || wideWeakBestLooksLikeFalseBrightMatch)
        && (!useStartDirection || !best.valid || wideWeakBestLooksLikeFalseBrightMatch)
        && (!blindSeedAlreadyAcceptable || wideWeakBestLooksLikeFalseBrightMatch)))
    {
        const qint64 stageStartMs = searchProfileTimer.elapsed();
        const std::array<double, 3> wideFovScales = {{0.70, 1.00, 1.30}};
        const std::array<double, 8> wideBlindFovs = {{15.0, 25.0, 40.0, 60.0, 90.0, 130.0, 160.0, 180.0}};
        const std::array<double, 3> wideAllSkyBlindFovs = {{130.0, 160.0, 180.0}};
        QVector<double> blindFallbackFovs;
        blindFallbackFovs.reserve(static_cast<int>(wideBlindFovs.size()) + 3);
        if (settings.m_fov < 15.0)
        {
            const auto& seedFovs = useWideBlindSeedRadius ? wideAllSkyBlindFovs : wideFovScales;
            for (double fovScale : seedFovs)
            {
                const double fovDegrees = useWideBlindSeedRadius
                    ? fovScale
                    : static_cast<double>(settings.m_fov) * fovScale;
                blindFallbackFovs.append(std::clamp(fovDegrees,
                    static_cast<double>(CameraSettings::m_minFov),
                    static_cast<double>(CameraSettings::m_maxFov)));
            }
        }
        if (!useWideBlindSeedRadius)
        {
            for (double fovDegrees : wideBlindFovs) {
                blindFallbackFovs.append(fovDegrees);
            }
        }
        const bool useNarrowBlindFallbackGrid = !useStartFov && (settings.m_fov < 15.0);
        const double fallbackAzimuthStepDegrees = useWideBlindSeedRadius ? 15.0
            : (useStartFov || useNarrowBlindFallbackGrid)
            ? fovGridAzimuthStepDegrees
            : azimuthStepDegrees;
        const double fallbackElevationStepDegrees = (useStartFov || useNarrowBlindFallbackGrid)
            ? fovGridElevationStepDegrees
            : elevationStepDegrees;
        // The effective match radius is constant across all grid points; compute
        // it once so populateBlindGridProjectedCatalog can apply the same bounds.
        const double blindMatchRadius = useStartFov
            ? (useWideFovSeedRadius  ? wideFovSeedMatchRadius
                                     : static_cast<double>(settings.m_plateSolveMatchRadius))
            : (useWideBlindSeedRadius ? wideBlindSeedMatchRadius
                                      : static_cast<double>(settings.m_plateSolveMatchRadius));

        // Roll-sweep optimisation: project the catalog ONCE per (Az, El, FOV)
        // at roll=0, cache the pixel offsets, then rotate by 2-D matrix for
        // each of the 13 roll values — ~13× fewer acos/atan2 calls per
        // Az×El×FOV cell.  Loop order is now FOV-outer, Roll-inner.
        for (double azimuthDegrees = minAzimuthDegrees; azimuthDegrees < maxAzimuthDegrees; azimuthDegrees += fallbackAzimuthStepDegrees)
        {
            if (isCancellationRequested()) break;
            for (double elevationDegrees = minElevationDegrees; elevationDegrees <= maxElevationDegrees; elevationDegrees += fallbackElevationStepDegrees)
            {
                if (isCancellationRequested()) break;
                if (useStartFov)
                {
                    for (double fovScale : wideFovScales)
                    {
                        if (isCancellationRequested()) break;
                        const double fovDegrees = std::clamp(
                            static_cast<double>(settings.m_fov) * fovScale,
                            static_cast<double>(CameraSettings::m_minFov),
                            static_cast<double>(CameraSettings::m_maxFov));
                        const SkyProjector refProjector = createProjector(
                            settings, imageSize,
                            azimuthDegrees, elevationDegrees, 0.0, fovDegrees,
                            fixedCenterOffsetX, fixedCenterOffsetY, fixedDistortionK1);
                        buildBlindGridCache(catalogContext, refProjector);
                        for (double rollDegrees : wideRollOffsets)
                        {
                            if (isCancellationRequested()) break;
                            populateBlindGridProjectedCatalog(rollDegrees, blindMatchRadius, refProjector);
                            evaluateSeedFromCache(
                                "wide-fallback-fov",
                                azimuthDegrees, elevationDegrees, rollDegrees,
                                fovDegrees, blindMatchRadius);
                        }
                    }
                }
                else
                {
                    for (double fovDegrees : blindFallbackFovs)
                    {
                        if (isCancellationRequested()) break;
                        const double clampedFov = std::clamp(
                            fovDegrees,
                            static_cast<double>(CameraSettings::m_minFov),
                            180.0);
                        const SkyProjector refProjector = createProjector(
                            settings, imageSize,
                            azimuthDegrees, elevationDegrees, 0.0, clampedFov,
                            fixedCenterOffsetX, fixedCenterOffsetY, fixedDistortionK1);
                        buildBlindGridCache(catalogContext, refProjector);
                        for (double rollDegrees : wideRollOffsets)
                        {
                            if (isCancellationRequested()) break;
                            populateBlindGridProjectedCatalog(rollDegrees, blindMatchRadius, refProjector);
                            evaluateSeedFromCache(
                                "wide-fallback-blind",
                                azimuthDegrees, elevationDegrees, rollDegrees,
                                clampedFov, blindMatchRadius);
                        }
                    }
                }
            }
        }
        logSearchProfile("wide-fallback-grid", stageStartMs);
    }

    if (!best.valid) {
        return best;
    }
    if (isCancellationRequested()) {
        return best;
    }

    double azCenter = best.azimuthDegrees;
    double elCenter = best.elevationDegrees;
    double rollCenter = best.rollDegrees;
    double fovCenter = best.fovDegrees;
    double azStep = std::max(0.5, coarseSearchRadius * 0.25);
    double elStep = azStep;
    double rollStep = std::max(1.0, coarseRollRadius * 0.25);
    double fovStep = std::max(minimumFovRefineStep, coarseFovRadius * 0.25);

    for (int iteration = 0; iteration < 2; ++iteration)
    {
        if (isCancellationRequested()) break;
        const qint64 stageStartMs = searchProfileTimer.elapsed();
        for (double azOffset : coarseFovOffsets)
        {
            if (isCancellationRequested()) break;
            for (double elOffset : coarseFovOffsets)
            {
                if (isCancellationRequested()) break;
                for (double rollOffset : coarseFovOffsets)
                {
                    if (isCancellationRequested()) break;
                    for (double fovOffset : coarseFovOffsets)
                    {
                        if (isCancellationRequested()) break;
                        const Evaluation candidate = evaluatePose(
                            settings,
                            catalogContext,
                            imageSize,
                            captureDateTimeUtc,
                            starDetections,
                            detectionIndices,
                            azCenter + azOffset * azStep,
                            elCenter + elOffset * elStep,
                            rollCenter + rollOffset * rollStep,
                            std::max(static_cast<double>(CameraSettings::m_minFov), fovCenter + fovOffset * fovStep),
                            nullptr,
                            fixedCenterOffsetX,
                            fixedCenterOffsetY,
                            fixedDistortionK1,
                            searchMatchRadiusOverride);
                        logPlateSolveEvaluation("coarse-refine", candidate);
                        if (keepMultipleCandidates) {
                            insertDistinctEvaluationCandidate(
                                *candidatePool,
                            candidate,
                            maxMultiHypothesisCandidates,
                            useWeakModeScoring,
                            "coarse-refine",
                            interestingWeakModeMatchCount,
                            weakModeCandidatePoolMinMatches,
                            useGuidedDirectionScoring);
                        }
                        if (isBetterEvaluationForMode(candidate, best, useWeakModeScoring, useGuidedDirectionScoring)) {
                            best = candidate;
                            logPlateSolveEvaluation("coarse-refine", best, true);
                        }
                    }
                }
            }
        }

        azCenter = best.azimuthDegrees;
        elCenter = best.elevationDegrees;
        rollCenter = best.rollDegrees;
        fovCenter = best.fovDegrees;
        azStep *= 0.5;
        elStep *= 0.5;
        rollStep *= 0.5;
        fovStep *= 0.5;
        logSearchProfile("coarse-refine", stageStartMs);
    }

    const QVector<int> allDetectionIndices = [&starDetections]() {
        QVector<int> indices;
        indices.reserve(starDetections.size());
        for (int i = 0; i < starDetections.size(); ++i) {
            indices.append(i);
        }
        return indices;
    }();
    const QVector<int>& fullRefineDetectionIndices =
        wideWeakMode ? detectionIndices : allDetectionIndices;

    azCenter = best.azimuthDegrees;
    elCenter = best.elevationDegrees;
    rollCenter = best.rollDegrees;
    fovCenter = best.fovDegrees;
    azStep = std::max(0.1, azStep);
    elStep = std::max(0.1, elStep);
    rollStep = std::max(0.25, rollStep);
    fovStep = std::max(minimumFovRefineStep, fovStep);

    for (int iteration = 0; iteration < 4; ++iteration)
    {
        if (isCancellationRequested()) break;
        const qint64 stageStartMs = searchProfileTimer.elapsed();
        bool improved = false;
        const std::array<double, 3> refineOffsets = {{-1.0, 0.0, 1.0}};
        for (double azOffset : refineOffsets)
        {
            if (isCancellationRequested()) break;
            for (double elOffset : refineOffsets)
            {
                if (isCancellationRequested()) break;
                for (double rollOffset : refineOffsets)
                {
                    if (isCancellationRequested()) break;
                    for (double fovOffset : refineOffsets)
                    {
                        if (isCancellationRequested()) break;
                        const Evaluation candidate = evaluatePose(
                            settings,
                            catalogContext,
                            imageSize,
                            captureDateTimeUtc,
                            starDetections,
                            fullRefineDetectionIndices,
                            azCenter + azOffset * azStep,
                            elCenter + elOffset * elStep,
                            rollCenter + rollOffset * rollStep,
                            std::max(static_cast<double>(CameraSettings::m_minFov), fovCenter + fovOffset * fovStep),
                            nullptr,
                            fixedCenterOffsetX,
                            fixedCenterOffsetY,
                            fixedDistortionK1,
                            searchMatchRadiusOverride);
                        logPlateSolveEvaluation("full-refine", candidate);
                        if (keepMultipleCandidates) {
                            insertDistinctEvaluationCandidate(
                                *candidatePool,
                            candidate,
                            maxMultiHypothesisCandidates,
                            useWeakModeScoring,
                            "full-refine",
                            interestingWeakModeMatchCount,
                            weakModeCandidatePoolMinMatches,
                            useGuidedDirectionScoring);
                        }
                        if (isBetterEvaluationForMode(candidate, best, useWeakModeScoring, useGuidedDirectionScoring)) {
                            best = candidate;
                            logPlateSolveEvaluation("full-refine", best, true);
                            improved = true;
                        }
                    }
                }
            }
        }

        azCenter = best.azimuthDegrees;
        elCenter = best.elevationDegrees;
        rollCenter = best.rollDegrees;
        fovCenter = best.fovDegrees;
        if (!improved) {
            azStep *= 0.5;
            elStep *= 0.5;
            rollStep *= 0.5;
            fovStep *= 0.5;
        }
        logSearchProfile("full-refine", stageStartMs);
    }

    return best;
}

CameraPlateSolver::SolverContext::PlateSolveLmEvaluation CameraPlateSolver::SolverContext::evaluateFixedPlateSolveLmPose(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<Match>& fixedMatches, const QVector<int>& rankDetectionIndices, const PlateSolveLmPose& inputPose, double robustThresholdPixels, const Evaluation& seedEvaluation, bool populateScoringMetrics)
{
    PlateSolveLmEvaluation lmEvaluation;
    lmEvaluation.pose = inputPose;
    clampPlateSolveLmPose(imageSize, lmEvaluation.pose);

    Evaluation& evaluation = lmEvaluation.evaluation;
    evaluation.azimuthDegrees = lmEvaluation.pose.azimuthDegrees;
    evaluation.elevationDegrees = lmEvaluation.pose.elevationDegrees;
    evaluation.rollDegrees = lmEvaluation.pose.rollDegrees;
    evaluation.fovDegrees = lmEvaluation.pose.fovDegrees;
    evaluation.centerOffsetXPixels = lmEvaluation.pose.centerOffsetXPixels;
    evaluation.centerOffsetYPixels = lmEvaluation.pose.centerOffsetYPixels;
    evaluation.distortionK1 = lmEvaluation.pose.distortionK1;
    evaluation.anchored = seedEvaluation.anchored;
    evaluation.sparseGuidedPair = seedEvaluation.sparseGuidedPair;
    evaluation.guidedTriangle = seedEvaluation.guidedTriangle;
    evaluation.anchorDetectionIndex = seedEvaluation.anchorDetectionIndex;
    evaluation.anchorCatalogIndex = seedEvaluation.anchorCatalogIndex;
    evaluation.secondaryAnchorDetectionIndex = seedEvaluation.secondaryAnchorDetectionIndex;
    evaluation.secondaryAnchorCatalogIndex = seedEvaluation.secondaryAnchorCatalogIndex;
    evaluation.tertiaryAnchorDetectionIndex = seedEvaluation.tertiaryAnchorDetectionIndex;
    evaluation.tertiaryAnchorCatalogIndex = seedEvaluation.tertiaryAnchorCatalogIndex;

    const SkyProjector projector = createProjector(
        settings,
        imageSize,
        evaluation.azimuthDegrees,
        evaluation.elevationDegrees,
        evaluation.rollDegrees,
        evaluation.fovDegrees,
        evaluation.centerOffsetXPixels,
        evaluation.centerOffsetYPixels,
        evaluation.distortionK1);
    if (!projector.valid) {
        return lmEvaluation;
    }

    QVector<ProjectedCatalogStar> projectedStars;
    projectedStars.reserve(fixedMatches.size());
    evaluation.matches.reserve(fixedMatches.size());
    lmEvaluation.residuals.reserve(fixedMatches.size() * 2);
    lmEvaluation.pairResidualNorms.reserve(fixedMatches.size());

    double sumSquaredError = 0.0;
    double robustCost = 0.0;
    const double thresholdPixels = std::max(1.0, robustThresholdPixels);
    for (const Match& fixedMatch : fixedMatches)
    {
        if ((fixedMatch.detectionIndex < 0)
            || (fixedMatch.detectionIndex >= starDetections.size())
            || (fixedMatch.catalogIndex < 0)
            || (fixedMatch.catalogIndex >= catalogContext.catalogStars.size()))
        {
            return lmEvaluation;
        }

        const auto visibleIt = catalogContext.visibleStarIndexByCatalogIndex.constFind(fixedMatch.catalogIndex);
        if (visibleIt == catalogContext.visibleStarIndexByCatalogIndex.cend()) {
            return lmEvaluation;
        }

        const VisibleCatalogStar& visibleStar = catalogContext.visibleStars[visibleIt.value()];
        QPointF projectedPoint;
        if (!projectVector(projector, visibleStar.vector, projectedPoint)) {
            return lmEvaluation;
        }

        const QPointF delta = projectedPoint - starDetections[fixedMatch.detectionIndex].m_center;
        const double distancePixels = std::hypot(delta.x(), delta.y());
        evaluation.matches.append({fixedMatch.detectionIndex, fixedMatch.catalogIndex, distancePixels});
        projectedStars.append({fixedMatch.catalogIndex, projectedPoint, visibleStar.magnitude});
        lmEvaluation.residuals.append(delta.x());
        lmEvaluation.residuals.append(delta.y());
        lmEvaluation.pairResidualNorms.append(distancePixels);
        sumSquaredError += distancePixels * distancePixels;
        robustCost += robustPlateSolveLmWeight(distancePixels, thresholdPixels) * distancePixels * distancePixels;
    }

    evaluation.matchCount = evaluation.matches.size();
    if (evaluation.matchCount <= 0) {
        return lmEvaluation;
    }

    evaluation.rmsErrorPixels = std::sqrt(sumSquaredError / evaluation.matchCount);
    evaluation.valid = true;
    // The brightness-rank/magnitude metrics drive candidate scoring, not the LM step (which uses
    // only residuals/robustCost). Skip them for the Jacobian finite-difference probes, which
    // discard everything but residuals -- they dominate the per-iteration eval count.
    if (populateScoringMetrics) {
        populatePoseScoringMetrics(
            settings,
            starDetections,
            rankDetectionIndices,
            projectedStars,
            catalogContext.catalogStars,
            evaluation);
    }
    lmEvaluation.valid = true;
    lmEvaluation.robustCost = robustCost;
    return lmEvaluation;
}

CameraPlateSolver::SolverContext::Evaluation CameraPlateSolver::SolverContext::runPlateSolveLmRefinement(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<Match>& fixedMatches, const QVector<int>& rankDetectionIndices, const Evaluation& seedEvaluation, std::array<bool, PlateSolveLmParameterCount> activeParameters, double matchRadiusPixels)
{
    if (fixedMatches.isEmpty()) {
        return seedEvaluation;
    }

    // WS2: rot-vec orientation parameterization, but only for direction-seeded (guided) solves --
    // blind/fov-only solves keep the legacy coordinate path (a global default regressed blind fisheye).
    const bool useRotVec = rotVecLmActive(settings);
    const int pairCount = fixedMatches.size();
    if (pairCount < 6)
    {
        activeParameters[PlateSolveLmCenterX] = false;
        activeParameters[PlateSolveLmCenterY] = false;
        activeParameters[PlateSolveLmDistortionK1] = false;
    }
    if (pairCount < 4) {
        activeParameters[PlateSolveLmFov] = false;
    }

    auto activeParameterList = [&activeParameters]() {
        QVector<int> active;
        active.reserve(PlateSolveLmParameterCount);
        for (int parameter = 0; parameter < PlateSolveLmParameterCount; ++parameter)
        {
            if (activeParameters[parameter]) {
                active.append(parameter);
            }
        }
        return active;
    };
    QVector<int> activeParametersList = activeParameterList();
    while ((activeParametersList.size() > pairCount * 2) && !activeParametersList.isEmpty())
    {
        const int parameter = activeParametersList.takeLast();
        activeParameters[parameter] = false;
    }
    if (activeParametersList.isEmpty()) {
        return evaluateFixedPlateSolveLmPose(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            fixedMatches,
            rankDetectionIndices,
            poseFromEvaluation(seedEvaluation),
            matchRadiusPixels,
            seedEvaluation).evaluation;
    }

    PlateSolveLmPose pose = poseFromEvaluation(seedEvaluation);
    clampPlateSolveLmPose(imageSize, pose);
    PlateSolveLmEvaluation best = evaluateFixedPlateSolveLmPose(
        settings,
        catalogContext,
        imageSize,
        starDetections,
        fixedMatches,
        rankDetectionIndices,
        pose,
        matchRadiusPixels,
        seedEvaluation);
    if (!best.valid) {
        return seedEvaluation;
    }

    double lambda = 1e-3;
    for (int iteration = 0; iteration < 15; ++iteration)
    {
        const double robustThresholdPixels = std::max(
            2.0,
            std::min(matchRadiusPixels, std::max(2.0, best.evaluation.rmsErrorPixels * 2.0)));
        best = evaluateFixedPlateSolveLmPose(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            fixedMatches,
            rankDetectionIndices,
            pose,
            robustThresholdPixels,
            seedEvaluation);
        if (!best.valid) {
            return seedEvaluation;
        }

        const int residualCount = best.residuals.size();
        QVector<QVector<double>> jacobianColumns;
        jacobianColumns.reserve(activeParametersList.size());
        bool jacobianValid = true;
        for (int parameterIndex : activeParametersList)
        {
            const PlateSolveLmParameter parameter = static_cast<PlateSolveLmParameter>(parameterIndex);
            double step = plateSolveLmFiniteDifferenceStep(pose, parameter);
            PlateSolveLmPose steppedPose = pose;
            addPlateSolveLmParameterDelta(imageSize, steppedPose, parameter, step, useRotVec);
            // WS2 rot-vec mode: for the orientation params the applied perturbation is a camera-frame
            // ROTATION of `step` degrees, not an az/el/roll coordinate addition. The Jacobian must be
            // taken w.r.t. that rotation angle (which is exactly what the update step applies via
            // addPlateSolveLmParameterDelta), NOT the resulting coordinate change -- the latter is
            // non-linear and degenerates to ~0 near zenith (collapsing the refinement). The legacy
            // coordinate-difference accounting is preserved verbatim when rot-vec mode is off.
            const bool rotVecOrientation = useRotVec
                && ((parameter == PlateSolveLmAzimuth)
                    || (parameter == PlateSolveLmElevation)
                    || (parameter == PlateSolveLmRoll));
            double appliedStep;
            if (rotVecOrientation)
            {
                appliedStep = step;
            }
            else
            {
                auto parameterDelta = [&pose, parameter](const PlateSolveLmPose& candidatePose) {
                    const double rawDelta = plateSolveLmParameterValue(candidatePose, parameter)
                        - plateSolveLmParameterValue(pose, parameter);
                    return ((parameter == PlateSolveLmAzimuth) || (parameter == PlateSolveLmRoll))
                        ? normalizeSignedDegrees(rawDelta)
                        : rawDelta;
                };
                appliedStep = parameterDelta(steppedPose);
                if (std::fabs(appliedStep) < 1e-12)
                {
                    steppedPose = pose;
                    addPlateSolveLmParameterDelta(imageSize, steppedPose, parameter, -step, useRotVec);
                    appliedStep = parameterDelta(steppedPose);
                }
                if (std::fabs(appliedStep) < 1e-12)
                {
                    jacobianValid = false;
                    break;
                }
            }

            const PlateSolveLmEvaluation stepped = evaluateFixedPlateSolveLmPose(
                settings,
                catalogContext,
                imageSize,
                starDetections,
                fixedMatches,
                rankDetectionIndices,
                steppedPose,
                robustThresholdPixels,
                seedEvaluation,
                false);  // Jacobian probe uses residuals only; skip scoring metrics.
            if (!stepped.valid || (stepped.residuals.size() != residualCount))
            {
                jacobianValid = false;
                break;
            }

            QVector<double> column;
            column.reserve(residualCount);
            for (int residualIndex = 0; residualIndex < residualCount; ++residualIndex) {
                column.append((stepped.residuals[residualIndex] - best.residuals[residualIndex]) / appliedStep);
            }
            jacobianColumns.append(column);
        }
        if (!jacobianValid) {
            break;
        }

        double normalMatrix[PlateSolveLmParameterCount][PlateSolveLmParameterCount] = {};
        double gradient[PlateSolveLmParameterCount] = {};
        const int activeCount = activeParametersList.size();
        for (int pairIndex = 0; pairIndex < pairCount; ++pairIndex)
        {
            const double weight = robustPlateSolveLmWeight(best.pairResidualNorms[pairIndex], robustThresholdPixels);
            for (int component = 0; component < 2; ++component)
            {
                const int residualIndex = pairIndex * 2 + component;
                for (int lhs = 0; lhs < activeCount; ++lhs)
                {
                    const double weightedJacobian = weight * jacobianColumns[lhs][residualIndex];
                    gradient[lhs] += weightedJacobian * best.residuals[residualIndex];
                    for (int rhs = lhs; rhs < activeCount; ++rhs) {
                        normalMatrix[lhs][rhs] += weightedJacobian * jacobianColumns[rhs][residualIndex];
                    }
                }
            }
        }
        for (int lhs = 0; lhs < activeCount; ++lhs)
        {
            for (int rhs = 0; rhs < lhs; ++rhs) {
                normalMatrix[lhs][rhs] = normalMatrix[rhs][lhs];
            }
        }

        double dampedMatrix[PlateSolveLmParameterCount][PlateSolveLmParameterCount] = {};
        double rhs[PlateSolveLmParameterCount] = {};
        for (int row = 0; row < activeCount; ++row)
        {
            rhs[row] = -gradient[row];
            for (int col = 0; col < activeCount; ++col) {
                dampedMatrix[row][col] = normalMatrix[row][col];
            }
            dampedMatrix[row][row] += lambda * std::max(std::fabs(normalMatrix[row][row]), 1e-9);
        }

        double delta[PlateSolveLmParameterCount] = {};
        if (!solveSmallLinearSystem(dampedMatrix, rhs, delta, activeCount))
        {
            lambda = std::min(lambda * 10.0, 1e12);
            continue;
        }

        PlateSolveLmPose proposedPose = pose;
        double maxNormalizedDelta = 0.0;
        for (int i = 0; i < activeCount; ++i)
        {
            const PlateSolveLmParameter parameter = static_cast<PlateSolveLmParameter>(activeParametersList[i]);
            const double maxStep = plateSolveLmMaximumStep(imageSize, pose, parameter);
            const double clampedDelta = std::clamp(delta[i], -maxStep, maxStep);
            addPlateSolveLmParameterDelta(imageSize, proposedPose, parameter, clampedDelta, useRotVec);
            maxNormalizedDelta = std::max(maxNormalizedDelta, std::fabs(clampedDelta) / std::max(maxStep, 1e-9));
        }

        const PlateSolveLmEvaluation proposed = evaluateFixedPlateSolveLmPose(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            fixedMatches,
            rankDetectionIndices,
            proposedPose,
            robustThresholdPixels,
            seedEvaluation);
        if (proposed.valid && (proposed.robustCost < best.robustCost))
        {
            const double previousCost = best.robustCost;
            pose = proposed.pose;
            best = proposed;
            lambda = std::max(lambda * 0.3, 1e-9);
            if ((maxNormalizedDelta < 1e-4)
                || ((best.robustCost > 0.0)
                    && ((previousCost - best.robustCost) / std::max(previousCost, 1.0) < 1e-8)))
            {
                break;
            }
        }
        else
        {
            lambda = std::min(lambda * 10.0, 1e12);
        }
    }

    return best.evaluation;
}

CameraPlateSolver::SolverContext::Evaluation CameraPlateSolver::SolverContext::refineGuidedAnchorSeedWithLm(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const QVector<int>& allowedCatalogIndices, const GuidedAnchorPair& anchor, const Evaluation& seedEvaluation, double centerOffsetXPixels, double centerOffsetYPixels, double distortionK1, double matchRadiusPixels, bool refineFov)
{
    QVector<Match> fixedMatches = uniqueValidMatchesForRefinement(
        catalogContext,
        starDetections,
        seedEvaluation.matches,
        &anchor);
    if (fixedMatches.isEmpty()) {
        return seedEvaluation;
    }

    std::array<bool, PlateSolveLmParameterCount> activeParameters = {{
        true,
        true,
        true,
        refineFov,
        false,
        false,
        false
    }};

    Evaluation best = seedEvaluation;
    for (int pass = 0; pass < 2; ++pass)
    {
        Evaluation lmEvaluation = runPlateSolveLmRefinement(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            fixedMatches,
            detectionIndices,
            best,
            activeParameters,
            matchRadiusPixels);
        if (isBetterGuidedAnchorEvaluation(lmEvaluation, best, matchRadiusPixels)) {
            best = lmEvaluation;
        }

        const Evaluation rematched = evaluateAnchoredPose(
            settings,
            catalogContext,
            imageSize,
            captureDateTimeUtc,
            starDetections,
            detectionIndices,
            allowedCatalogIndices,
            anchor,
            best.azimuthDegrees,
            best.elevationDegrees,
            best.rollDegrees,
            best.fovDegrees,
            centerOffsetXPixels,
            centerOffsetYPixels,
            distortionK1,
            matchRadiusPixels);
        if (isBetterGuidedAnchorEvaluation(rematched, best, matchRadiusPixels)) {
            best = rematched;
        }

        QVector<Match> rematchedFixedMatches = uniqueValidMatchesForRefinement(
            catalogContext,
            starDetections,
            best.matches,
            &anchor);
        if (rematchedFixedMatches.size() <= fixedMatches.size()) {
            break;
        }
        fixedMatches = rematchedFixedMatches;
    }

    return best;
}

QVector<CameraPlateSolver::SolverContext::Match> CameraPlateSolver::SolverContext::rebuildRefinementMatchesAtPose(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const Evaluation& pose, double matchRadiusPixels, const GuidedAnchorPair *forcedAnchor)
{
    const SkyProjector projector = createProjector(
        settings,
        imageSize,
        pose.azimuthDegrees,
        pose.elevationDegrees,
        pose.rollDegrees,
        pose.fovDegrees,
        pose.centerOffsetXPixels,
        pose.centerOffsetYPixels,
        pose.distortionK1);
    if (!projector.valid) {
        return QVector<Match>();
    }

    QVector<ProjectedCatalogStar> projectedStars = buildProjectedCatalog(
        catalogContext,
        projector,
        matchRadiusPixels);
    if (projectedStars.isEmpty()) {
        return QVector<Match>();
    }

    QVector<Match> matches = buildMatches(
        catalogContext,
        starDetections,
        detectionIndices,
        projectedStars,
        matchRadiusPixels);

    if (forcedAnchor
        && (forcedAnchor->detectionIndex >= 0)
        && (forcedAnchor->detectionIndex < starDetections.size()))
    {
        int anchorProjectedIndex = -1;
        for (int i = 0; i < projectedStars.size(); ++i)
        {
            if (projectedStars[i].catalogIndex == forcedAnchor->catalogIndex)
            {
                anchorProjectedIndex = i;
                break;
            }
        }
        if (anchorProjectedIndex >= 0)
        {
            const double anchorDistance = pointDistancePixels(
                starDetections[forcedAnchor->detectionIndex].m_center,
                projectedStars[anchorProjectedIndex].point);
            if (anchorDistance <= matchRadiusPixels)
            {
                QVector<Match> anchoredMatches;
                anchoredMatches.reserve(matches.size() + 1);
                anchoredMatches.append({
                    forcedAnchor->detectionIndex,
                    forcedAnchor->catalogIndex,
                    anchorDistance
                });
                QSet<int> matchedDetections;
                QSet<int> matchedCatalogStars;
                matchedDetections.insert(forcedAnchor->detectionIndex);
                matchedCatalogStars.insert(forcedAnchor->catalogIndex);
                for (const Match& match : matches)
                {
                    if (matchedDetections.contains(match.detectionIndex)
                        || matchedCatalogStars.contains(match.catalogIndex))
                    {
                        continue;
                    }
                    matchedDetections.insert(match.detectionIndex);
                    matchedCatalogStars.insert(match.catalogIndex);
                    anchoredMatches.append(match);
                }
                matches = anchoredMatches;
            }
        }
    }

    int outlierCount = 0;
    matches = rejectOutlierMatches(
        matches,
        settings.m_plateSolveMinMatches,
        matchRadiusPixels,
        &outlierCount);
    appendSupplementalMatches(
        starDetections,
        projectedStars,
        matchRadiusPixels,
        &detectionIndices,
        m_useDirectionSeedPreference && (isNarrowField(settings)),
        false,   // tight-artifact coincidence is applied label-only post-acceptance, not during scoring
        matches);
    appendWideBrightSupplementalMatches(
        settings,
        starDetections,
        projectedStars,
        imageSize,
        matchRadiusPixels,
        matches);
    return matches;
}

CameraPlateSolver::SolverContext::Evaluation CameraPlateSolver::SolverContext::refinePoseFromMatches(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const Evaluation& initialEvaluation, bool forceFovRefine)
{
    Q_UNUSED(captureDateTimeUtc)

    if (!initialEvaluation.valid || initialEvaluation.matches.isEmpty()) {
        return initialEvaluation;
    }
    const bool calibrateLens = canCalibrateLens(settings);
    const bool calibratePrincipalPoint = canCalibratePrincipalPoint(settings);
    const bool useGuidedDirectionScoring = plateSolveStartUsesDirection(settings);

    const QVector<Match> inlierMatches = rejectOutlierMatches(
        initialEvaluation.matches,
        settings.m_plateSolveMinMatches,
        settings.m_plateSolveMatchRadius,
        nullptr);

    const bool forceAnchor = initialEvaluation.anchored
        && (initialEvaluation.anchorDetectionIndex >= 0)
        && (initialEvaluation.anchorCatalogIndex >= 0);
    GuidedAnchorPair forcedAnchor;
    if (forceAnchor)
    {
        forcedAnchor.detectionIndex = initialEvaluation.anchorDetectionIndex;
        forcedAnchor.catalogIndex = initialEvaluation.anchorCatalogIndex;
        for (const Match& match : initialEvaluation.matches)
        {
            if ((match.detectionIndex == forcedAnchor.detectionIndex)
                && (match.catalogIndex == forcedAnchor.catalogIndex))
            {
                forcedAnchor.initialDistancePixels = match.distancePixels;
                break;
            }
        }
    }

    Evaluation seed = initialEvaluation;
    seed.centerOffsetXPixels = calibratePrincipalPoint ? initialEvaluation.centerOffsetXPixels : settings.m_lensCenterOffsetX;
    seed.centerOffsetYPixels = calibratePrincipalPoint ? initialEvaluation.centerOffsetYPixels : settings.m_lensCenterOffsetY;
    seed.distortionK1 = calibrateLens ? initialEvaluation.distortionK1 : settings.m_lensDistortionK1;

    QVector<Match> fixedMatches = uniqueValidMatchesForRefinement(
        catalogContext,
        starDetections,
        inlierMatches,
        forceAnchor ? &forcedAnchor : nullptr);
    if (fixedMatches.isEmpty()) {
        return seed;
    }

    QVector<int> rankDetectionIndices = detectionIndicesForMatches(fixedMatches);
    std::array<bool, PlateSolveLmParameterCount> activeParameters = {{
        true,
        true,
        true,
        forceFovRefine || !(useGuidedDirectionScoring && isNarrowField(settings)),
        calibratePrincipalPoint,
        calibratePrincipalPoint,
        calibrateLens
    }};

    Evaluation best = runPlateSolveLmRefinement(
        settings,
        catalogContext,
        imageSize,
        starDetections,
        fixedMatches,
        rankDetectionIndices,
        seed,
        activeParameters,
        static_cast<double>(settings.m_plateSolveMatchRadius));
    if (!best.valid) {
        best = seed;
    }

    QVector<int> expandedDetectionIndices = selectDetectionIndicesForSolve(starDetections, imageSize);
    QSet<int> expandedDetectionSet;
    expandedDetectionSet.reserve(expandedDetectionIndices.size() + rankDetectionIndices.size());
    for (int detectionIndex : expandedDetectionIndices) {
        expandedDetectionSet.insert(detectionIndex);
    }
    for (int detectionIndex : rankDetectionIndices)
    {
        if (!expandedDetectionSet.contains(detectionIndex))
        {
            expandedDetectionSet.insert(detectionIndex);
            expandedDetectionIndices.append(detectionIndex);
        }
    }

    const double finalPassRadius = std::min(
        static_cast<double>(settings.m_plateSolveMatchRadius),
        std::max(1.0, static_cast<double>(settings.m_plateSolveFinalMatchRadius)));
    const double intermediatePassRadius = std::min(
        static_cast<double>(settings.m_plateSolveMatchRadius),
        std::max(finalPassRadius + 10.0, finalPassRadius * 2.0));

    const QVector<Match> rebuiltMatches = rebuildRefinementMatchesAtPose(
        settings,
        catalogContext,
        imageSize,
        starDetections,
        expandedDetectionIndices,
        best,
        intermediatePassRadius,
        forceAnchor ? &forcedAnchor : nullptr);
    if (!rebuiltMatches.isEmpty())
    {
        QVector<Match> rebuiltFixedMatches = uniqueValidMatchesForRefinement(
            catalogContext,
            starDetections,
            rebuiltMatches,
            forceAnchor ? &forcedAnchor : nullptr);
        if (rebuiltFixedMatches.size() >= fixedMatches.size())
        {
            Evaluation expandedSeed = best;
            expandedSeed.matches = rebuiltFixedMatches;
            expandedSeed.matchCount = rebuiltFixedMatches.size();
            const Evaluation expandedBest = runPlateSolveLmRefinement(
                settings,
                catalogContext,
                imageSize,
                starDetections,
                rebuiltFixedMatches,
                expandedDetectionIndices,
                expandedSeed,
                activeParameters,
                intermediatePassRadius);
            if (isBetterEvaluationForMode(expandedBest, best, false, useGuidedDirectionScoring)) {
                best = expandedBest;
            }
        }
    }

    const int retainedMatchThreshold = forceAnchor
        ? std::max(2, std::min(settings.m_plateSolveMinMatches, best.matchCount))
        : minimumRetainedMatchesForFinalPass(best, settings.m_plateSolveMinMatches);
    const FinalMatchPassEvaluation finalPass = evaluateFinalMatchPass(
        settings,
        catalogContext,
        imageSize,
        starDetections,
        expandedDetectionIndices,
        best,
        finalPassRadius);
    if (finalPass.projectorValid && (finalPass.finalMatches.size() >= retainedMatchThreshold))
    {
        Evaluation finalEvaluation = best;
        finalEvaluation.matches = finalPass.finalMatches;
        finalEvaluation.matchCount = finalPass.finalMatches.size();
        finalEvaluation.rmsErrorPixels = finalPass.rmsErrorPixels;
        finalEvaluation.brightnessRankError = finalPass.brightnessRankError;
        finalEvaluation.meanCatalogMagnitude = finalPass.meanCatalogMagnitude;
        finalEvaluation.valid = true;
        if (isBetterFinalPassEvaluation(finalEvaluation, best, retainedMatchThreshold, useGuidedDirectionScoring)
            || (finalEvaluation.matchCount >= best.matchCount))
        {
            best = finalEvaluation;
        }
    }

    if (!calibratePrincipalPoint)
    {
        best.centerOffsetXPixels = settings.m_lensCenterOffsetX;
        best.centerOffsetYPixels = settings.m_lensCenterOffsetY;
    }
    if (!calibrateLens)
    {
        best.distortionK1 = settings.m_lensDistortionK1;
    }

    return best;
}

CameraPlateSolver::SolverContext::FinalMatchPassEvaluation CameraPlateSolver::SolverContext::tightenNarrowFinalPass(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const FinalMatchPassEvaluation& original, double finalMatchRadius)
{
    const int minMatches = std::max(4, settings.m_plateSolveMinMatches);
    if (!original.projectorValid
        || (original.finalMatches.size() < minMatches))
    {
        return original;
    }

    Evaluation pose = original.pose;
    for (const double fraction : {0.5, 0.33})
    {
        const double tightRadius = std::max(4.0, finalMatchRadius * fraction);
        const FinalMatchPassEvaluation tightPass = evaluateFinalMatchPass(
            settings, catalogContext, imageSize, starDetections, detectionIndices, pose, tightRadius);
        if (!tightPass.projectorValid || (tightPass.finalMatches.size() < minMatches)) {
            break;
        }
        Evaluation tightSeed = pose;
        tightSeed.valid = true;
        tightSeed.matches = tightPass.finalMatches;
        tightSeed.matchCount = static_cast<int>(tightPass.finalMatches.size());
        const Evaluation refined = refinePoseFromMatches(
            settings, catalogContext, imageSize, QDateTime(), starDetections, tightSeed);
        if (refined.valid) {
            pose = refined;
        }
    }

    const FinalMatchPassEvaluation tightened = evaluateFinalMatchPass(
        settings, catalogContext, imageSize, starDetections, detectionIndices, pose, finalMatchRadius);
    // Guard against the tightened fit drifting to a different FOV that happens to
    // have a slightly lower RMS on its own (smaller) inlier set but is a worse pose
    // overall -- only trust the tightened result if its FOV stayed close to the
    // pose it was sharpening.
    const bool fovDriftAcceptable = !std::isfinite(original.pose.fovDegrees)
        || (original.pose.fovDegrees <= 0.0)
        || (std::abs(tightened.pose.fovDegrees - original.pose.fovDegrees)
            <= std::max(5.0, original.pose.fovDegrees * 0.05));
    if (tightened.projectorValid
        && std::isfinite(tightened.rmsErrorPixels)
        && (tightened.finalMatches.size()
            >= static_cast<qsizetype>(std::floor(static_cast<double>(original.finalMatches.size()) * 0.9)))
        && (tightened.rmsErrorPixels <= original.rmsErrorPixels)
        && fovDriftAcceptable)
    {
        return tightened;
    }
    return original;
}

CameraPlateSolver::SolverContext::CandidateRefinementResult CameraPlateSolver::SolverContext::refineMultiHypothesisCandidate(const CameraSettings& settings, const PlateSolveCatalogContext& catalogContext, const QSize& imageSize, const QDateTime& captureDateTimeUtc, const QVector<CameraPipelineStarDetection>& starDetections, const QVector<int>& detectionIndices, const QVector<int>& allDetectionIndices, const Evaluation& candidate, int weakModeRefineMinMatches, double finalMatchRadius, bool rankFinalPassWithSelectedDetections, bool useStartDirection)
{
    CandidateRefinementResult result;
    const int candidateRefineMinMatches = (candidate.anchored || candidate.sparseGuidedPair)
        ? 2
        : weakModeRefineMinMatches;
    if (!candidate.valid || (candidate.matchCount < candidateRefineMinMatches)) {
        return result;
    }

    if (candidate.sparseGuidedPair || candidate.guidedTriangle)
    {
        result.seedFinalPassEvaluation = evaluateFinalMatchPass(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            allDetectionIndices,
            candidate,
            finalMatchRadius);
        result.seedFinalPassEvaluated = true;
    }

    result.refinedCandidate = refinePoseFromMatches(
        settings,
        catalogContext,
        imageSize,
        captureDateTimeUtc,
        starDetections,
        candidate);
    result.refinedCandidateEvaluated = true;
    if (!result.refinedCandidate.valid
        || (result.refinedCandidate.matchCount < candidateRefineMinMatches))
    {
        return result;
    }

    // For the narrow dense-field case the full-detection pass overrides the
    // selected-detection pass whenever it is valid (the common case), so compute the
    // full pass first and only fall back to the selected-detection pass when needed.
    if (rankFinalPassWithSelectedDetections
        && useStartDirection
        && (isNarrowField(settings)))
    {
        result.finalPassEvaluation = evaluateFinalMatchPass(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            allDetectionIndices,
            result.refinedCandidate,
            finalMatchRadius);
        if (!result.finalPassEvaluation.projectorValid)
        {
            result.finalPassEvaluation = evaluateFinalMatchPass(
                settings,
                catalogContext,
                imageSize,
                starDetections,
                detectionIndices,
                result.refinedCandidate,
                finalMatchRadius,
                rankFinalPassWithSelectedDetections);
        }
    }
    else
    {
        result.finalPassEvaluation = evaluateFinalMatchPass(
            settings,
            catalogContext,
            imageSize,
            starDetections,
            rankFinalPassWithSelectedDetections ? detectionIndices : allDetectionIndices,
            result.refinedCandidate,
            finalMatchRadius,
            rankFinalPassWithSelectedDetections);
    }
    result.finalPassEvaluated = true;
    return result;
}

