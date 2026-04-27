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

#ifndef INCLUDE_FEATURE_CAMERAFINDER_H_
#define INCLUDE_FEATURE_CAMERAFINDER_H_

#include <QObject>
#include <QStringList>

#include "camerasettings.h"

class MessageQueue;
class QNetworkAccessManager;

class CameraFinder : public QObject
{
    Q_OBJECT
public:
    explicit CameraFinder(QObject* parent = nullptr);
    ~CameraFinder() override;

    void setMessageQueueToGUI(MessageQueue* messageQueue) { m_msgQueueToGUI = messageQueue; }
    void reportCameraList(const CameraSettings& settings);

private:
    MessageQueue* m_msgQueueToGUI;
    QNetworkAccessManager* m_networkManager;

    static QStringList listQtCameraIds();
    static QStringList parseAlpacaCameraList(const QByteArray& payload);
    static QString buildAlpacaBaseUrl(const CameraSettings& settings);
};

#endif // INCLUDE_FEATURE_CAMERAFINDER_H_
