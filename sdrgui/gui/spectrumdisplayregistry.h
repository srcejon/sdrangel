///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston <jon@beniston.com>                            //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License.                     //
///////////////////////////////////////////////////////////////////////////////////

#ifndef SDRGUI_SPECTRUMDISPLAYREGISTRY_H_
#define SDRGUI_SPECTRUMDISPLAYREGISTRY_H_

#include <QImage>
#include <QList>
#include <QObject>
#include <QPointer>
#include <QString>

#include "export.h"

class GLSpectrumView;

struct SDRGUI_API SpectrumDisplaySourceInfo
{
    QString m_id;
    QString m_displayName;

    bool operator==(const SpectrumDisplaySourceInfo& other) const
    {
        return (m_id == other.m_id) && (m_displayName == other.m_displayName);
    }
};

class SDRGUI_API SpectrumDisplayRegistry : public QObject
{
    Q_OBJECT

public:
    explicit SpectrumDisplayRegistry(QObject *parent = nullptr);

    void registerSource(
        QObject *owner,
        const QString& role,
        const QString& title,
        GLSpectrumView *view);
    void unregisterSource(GLSpectrumView *view);

    QList<SpectrumDisplaySourceInfo> sources() const;

    // The response is always asynchronous, including cache hits and errors.
    quint64 requestImage(const QString& sourceId, int maximumAgeMs);

signals:
    void sourcesChanged(const QStringList& renameFrom, const QStringList& renameTo);
    void imageReady(quint64 requestId, const QString& sourceId, const QImage& image);

private:
    struct PendingRequest
    {
        quint64 m_requestId = 0;
        QString m_sourceId;
    };

    struct Source
    {
        QPointer<QObject> m_owner;
        QPointer<QObject> m_view;
        QString m_role;
        QString m_title;
        QString m_id;
        QString m_displayName;
        QImage m_cachedImage;
        qint64 m_lastCaptureMs = 0;
        bool m_capturePending = false;
        QList<PendingRequest> m_pendingRequests;
    };

    QList<Source> m_sources;
    quint64 m_nextRequestId = 1;

    int findSourceById(const QString& sourceId) const;
    int findSourceByView(const GLSpectrumView *view) const;
    QString ownerLongId(const QObject *owner) const;
    void updateSourceIdentity(Source& source) const;
    void refreshSourceIdentities();
    void captureSource(QObject *viewObject);
    void failPendingRequests(Source& source);
};

#endif // SDRGUI_SPECTRUMDISPLAYREGISTRY_H_
