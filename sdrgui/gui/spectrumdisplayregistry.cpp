///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston <jon@beniston.com>                            //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License.                     //
///////////////////////////////////////////////////////////////////////////////////

#include <algorithm>

#include <QDateTime>
#include <QTimer>

#include "availablechannelorfeature.h"
#include "availabledevice.h"
#include "maincore.h"

#include "glspectrumview.h"
#include "spectrumdisplayregistry.h"

SpectrumDisplayRegistry::SpectrumDisplayRegistry(QObject *parent) :
    QObject(parent)
{
    MainCore *mainCore = MainCore::instance();
    connect(mainCore, &MainCore::deviceSetAdded, this, [this](int, DeviceAPI *) {
        refreshSourceIdentities();
    });
    connect(mainCore, &MainCore::deviceSetRemoved, this, [this](int) {
        refreshSourceIdentities();
    });
    connect(mainCore, &MainCore::channelAdded, this, [this](int, ChannelAPI *) {
        refreshSourceIdentities();
    });
    connect(mainCore, &MainCore::channelRemoved, this, [this](int, ChannelAPI *) {
        refreshSourceIdentities();
    });
    connect(mainCore, &MainCore::featureAdded, this, [this](int, Feature *) {
        refreshSourceIdentities();
    });
    connect(mainCore, &MainCore::featureRemoved, this, [this](int, Feature *) {
        refreshSourceIdentities();
    });
}

void SpectrumDisplayRegistry::registerSource(
    QObject *owner,
    const QString& role,
    const QString& title,
    GLSpectrumView *view)
{
    if (!owner || !view) {
        return;
    }

    unregisterSource(view);

    Source source;
    source.m_owner = owner;
    source.m_view = view;
    source.m_role = role.trimmed().isEmpty() ? QStringLiteral("main") : role.trimmed();
    source.m_title = title.trimmed();
    updateSourceIdentity(source);
    m_sources.append(source);

    connect(view, &QObject::destroyed, this, [this]() {
        bool changed = false;
        for (int i = m_sources.size() - 1; i >= 0; --i)
        {
            if (!m_sources.at(i).m_view)
            {
                failPendingRequests(m_sources[i]);
                m_sources.removeAt(i);
                changed = true;
            }
        }
        if (changed) {
            emit sourcesChanged({}, {});
        }
    });
    connect(owner, &QObject::destroyed, this, [this, owner]() {
        bool changed = false;

        for (int i = m_sources.size() - 1; i >= 0; --i)
        {
            if (!m_sources.at(i).m_owner || (m_sources.at(i).m_owner.data() == owner))
            {
                failPendingRequests(m_sources[i]);
                m_sources.removeAt(i);
                changed = true;
            }
        }

        if (changed) {
            emit sourcesChanged({}, {});
        }
    });

    emit sourcesChanged({}, {});
    QTimer::singleShot(0, this, &SpectrumDisplayRegistry::refreshSourceIdentities);
}

void SpectrumDisplayRegistry::unregisterSource(GLSpectrumView *view)
{
    const int index = findSourceByView(view);
    if (index < 0) {
        return;
    }

    failPendingRequests(m_sources[index]);
    m_sources.removeAt(index);
    emit sourcesChanged({}, {});
}

QList<SpectrumDisplaySourceInfo> SpectrumDisplayRegistry::sources() const
{
    QList<SpectrumDisplaySourceInfo> result;
    result.reserve(m_sources.size());

    for (const Source& source : m_sources)
    {
        if (!source.m_id.isEmpty() && source.m_view) {
            result.append({source.m_id, source.m_displayName});
        }
    }

    std::sort(result.begin(), result.end(), [](const SpectrumDisplaySourceInfo& lhs, const SpectrumDisplaySourceInfo& rhs) {
        return lhs.m_displayName.localeAwareCompare(rhs.m_displayName) < 0;
    });
    return result;
}

quint64 SpectrumDisplayRegistry::requestImage(const QString& sourceId, int maximumAgeMs)
{
    const quint64 requestId = m_nextRequestId++;
    const int index = findSourceById(sourceId);

    if (index < 0)
    {
        QTimer::singleShot(0, this, [this, requestId, sourceId]() {
            emit imageReady(requestId, sourceId, QImage());
        });
        return requestId;
    }

    Source& source = m_sources[index];
    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    const bool cacheFresh = !source.m_cachedImage.isNull()
        && (maximumAgeMs > 0)
        && ((nowMs - source.m_lastCaptureMs) <= maximumAgeMs);

    if (cacheFresh)
    {
        const QImage cachedImage = source.m_cachedImage;
        QTimer::singleShot(0, this, [this, requestId, sourceId, cachedImage]() {
            emit imageReady(requestId, sourceId, cachedImage);
        });
        return requestId;
    }

    source.m_pendingRequests.append({requestId, sourceId});
    if (!source.m_capturePending)
    {
        source.m_capturePending = true;
        QPointer<QObject> view = source.m_view;
        QTimer::singleShot(0, this, [this, view]() {
            captureSource(view.data());
        });
    }

    return requestId;
}

int SpectrumDisplayRegistry::findSourceById(const QString& sourceId) const
{
    for (int i = 0; i < m_sources.size(); ++i)
    {
        if (m_sources.at(i).m_id == sourceId) {
            return i;
        }
    }

    return -1;
}

int SpectrumDisplayRegistry::findSourceByView(const GLSpectrumView *view) const
{
    for (int i = 0; i < m_sources.size(); ++i)
    {
        if (m_sources.at(i).m_view.data() == view) {
            return i;
        }
    }

    return -1;
}

QString SpectrumDisplayRegistry::ownerLongId(const QObject *owner) const
{
    if (!owner) {
        return QString();
    }

    const AvailableDeviceList devices = MainCore::instance()->getAvailableDevices({}, QStringLiteral("RTM"));
    const int deviceIndex = devices.indexOfObject(owner);
    if (deviceIndex >= 0) {
        return devices.at(deviceIndex).getLongId();
    }

    const AvailableChannelOrFeatureList channelsAndFeatures =
        MainCore::instance()->getAvailableChannelsAndFeatures({}, QStringLiteral("RTMF"));
    const int channelOrFeatureIndex = channelsAndFeatures.indexOfObject(owner);
    if (channelOrFeatureIndex >= 0) {
        return channelsAndFeatures.at(channelOrFeatureIndex).getLongId();
    }

    return QString();
}

void SpectrumDisplayRegistry::updateSourceIdentity(Source& source) const
{
    const QString ownerId = ownerLongId(source.m_owner.data());
    if (ownerId.isEmpty())
    {
        source.m_id.clear();
        source.m_displayName.clear();
        return;
    }

    if (source.m_role == QLatin1String("main"))
    {
        // Preserve the existing device spectrum identifier so saved camera
        // spectrum-overlay selections continue to resolve.
        source.m_id = ownerId;
        source.m_displayName = ownerId;
    }
    else
    {
        source.m_id = ownerId + QLatin1Char('|') + source.m_role;
        source.m_displayName = source.m_title.isEmpty()
            ? source.m_id
            : ownerId + QStringLiteral(" - ") + source.m_title;
    }
}

void SpectrumDisplayRegistry::refreshSourceIdentities()
{
    QStringList renameFrom;
    QStringList renameTo;
    bool changed = false;

    for (Source& source : m_sources)
    {
        const QString oldId = source.m_id;
        const QString oldDisplayName = source.m_displayName;
        updateSourceIdentity(source);

        if (oldId != source.m_id)
        {
            if (!oldId.isEmpty() && !source.m_id.isEmpty())
            {
                renameFrom.append(oldId);
                renameTo.append(source.m_id);
            }
            changed = true;
        }
        else if (oldDisplayName != source.m_displayName)
        {
            changed = true;
        }
    }

    if (changed) {
        emit sourcesChanged(renameFrom, renameTo);
    }
}

void SpectrumDisplayRegistry::captureSource(QObject *viewObject)
{
    GLSpectrumView *view = qobject_cast<GLSpectrumView *>(viewObject);
    const int index = findSourceByView(view);
    if (index < 0) {
        return;
    }

    Source& source = m_sources[index];
    QImage image;

    if (view && view->isValid())
    {
        image = view->grabFramebuffer();
        if (!image.isNull()) {
            image.setDevicePixelRatio(std::max(1.0, static_cast<double>(view->devicePixelRatioF())));
        }
    }

    if (!image.isNull())
    {
        source.m_cachedImage = image;
        source.m_lastCaptureMs = QDateTime::currentMSecsSinceEpoch();
    }

    const QList<PendingRequest> requests = source.m_pendingRequests;
    source.m_pendingRequests.clear();
    source.m_capturePending = false;

    for (const PendingRequest& request : requests) {
        emit imageReady(request.m_requestId, request.m_sourceId, image);
    }
}

void SpectrumDisplayRegistry::failPendingRequests(Source& source)
{
    const QList<PendingRequest> requests = source.m_pendingRequests;
    source.m_pendingRequests.clear();
    source.m_capturePending = false;

    for (const PendingRequest& request : requests) {
        emit imageReady(request.m_requestId, request.m_sourceId, QImage());
    }
}
