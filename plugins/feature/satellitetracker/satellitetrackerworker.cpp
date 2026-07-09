///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2021-2023 Jon Beniston, M7RCE <jon@beniston.com>                //
// Copyright (C) 2021-2022 Edouard Griffiths, F4EXB <f4exb06@gmail.com>          //
// Copyright (C) 2022 Jiří Pinkava <jiri.pinkava@rossum.ai>                      //
// Copyright (C) 2023 Daniele Forsi <iu5hkx@gmail.com>                           //
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

#include <climits>
#include <cmath>

#include <QDebug>
#include <QAbstractSocket>
#include <QTcpServer>
#include <QTcpSocket>
#include <QEventLoop>
#include <QTimer>
#include <QDateTime>

#include "SWGTargetAzimuthElevation.h"
#include "SWGMapItem.h"

#include "webapi/webapiadapterinterface.h"

#include "util/units.h"
#include "util/profiler.h"
#include "device/deviceset.h"
#include "device/deviceapi.h"
#include "channel/channelapi.h"
#include "channel/channelwebapiutils.h"
#include "feature/featurewebapiutils.h"
#include "maincore.h"

#include "satellitetracker.h"
#include "satellitetrackerworker.h"
#include "satellitetrackerreport.h"
#include "satellitetrackersgp4.h"

MESSAGE_CLASS_DEFINITION(SatelliteTrackerWorker::MsgConfigureSatelliteTrackerWorker, Message)
MESSAGE_CLASS_DEFINITION(SatelliteTrackerReport::MsgReportSat, Message)
MESSAGE_CLASS_DEFINITION(SatelliteTrackerReport::MsgReportAOS, Message)
MESSAGE_CLASS_DEFINITION(SatelliteTrackerReport::MsgReportLOS, Message)
MESSAGE_CLASS_DEFINITION(SatelliteTrackerReport::MsgReportTarget, Message)

// We don't want to send the passes and ground track to the GUI thread as it is not needed there and it is a lot of data.
static SatelliteState createSatelliteReportState(const SatelliteState& source)
{
    SatelliteState report;
    report.m_name = source.m_name;
    report.m_latitude = source.m_latitude;
    report.m_longitude = source.m_longitude;
    report.m_altitude = source.m_altitude;
    report.m_azimuth = source.m_azimuth;
    report.m_elevation = source.m_elevation;
    report.m_range = source.m_range;
    report.m_rangeRate = source.m_rangeRate;
    report.m_speed = source.m_speed;
    report.m_period = source.m_period;
    report.m_error = source.m_error;
    report.m_passes = source.m_passes;
    return report;
}

SatelliteTrackerWorker::SatelliteTrackerWorker(SatelliteTracker* satelliteTracker, WebAPIAdapterInterface *webAPIAdapterInterface) :
    m_satelliteTracker(satelliteTracker),
    m_webAPIAdapterInterface(webAPIAdapterInterface),
    m_msgQueueToFeature(nullptr),
    m_msgQueueToGUI(nullptr),
    m_pollTimer(this),
    m_schedulerTimer(this),
    m_recalculatePasses(true),
    m_flipRotation(false),
    m_extendedAzRotation(false),
    m_running(false)
{
    connect(&m_pollTimer, SIGNAL(timeout()), this, SLOT(update()));
    connect(&m_schedulerTimer, &QTimer::timeout, this, &SatelliteTrackerWorker::schedulerTick);
    m_schedulerTimer.setSingleShot(true);
}

SatelliteTrackerWorker::~SatelliteTrackerWorker()
{
    qDebug() << "SatelliteTrackerWorker::~SatelliteTrackerWorker";
    stopWork();
    m_inputMessageQueue.clear();
    // Remove satellites from Map
    QHashIterator<QString, SatWorkerState *> itr(m_workerState);
    while (itr.hasNext())
    {
        itr.next();
        if (m_settings.m_drawOnMap) {
            removeFromMap(itr.key());
        }
    }
    qDeleteAll(m_workerState);
}

void SatelliteTrackerWorker::startWork()
{
    qDebug() << "SatelliteTrackerWorker::startWork";
    QMutexLocker mutexLocker(&m_mutex);
    m_running = true;
    connect(&m_inputMessageQueue, SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));
    m_recalculatePasses = true;

     m_pollTimer.start((int)round(m_settings.m_updatePeriod*1000.0));

    // Handle any messages already on the queue
    handleInputMessages();
    rescheduleTimer();
}

void SatelliteTrackerWorker::stopWork()
{
    qDebug() << "SatelliteTrackerWorker::stopWork";
    QMutexLocker mutexLocker(&m_mutex);
    m_running = false;
    disconnect(&m_inputMessageQueue, SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));
    m_pollTimer.stop();
    m_schedulerTimer.stop();
}

void SatelliteTrackerWorker::handleInputMessages()
{
    Message* message;

    while ((message = m_inputMessageQueue.pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

bool SatelliteTrackerWorker::handleMessage(const Message& message)
{
    if (MsgConfigureSatelliteTrackerWorker::match(message))
    {
        QMutexLocker mutexLocker(&m_mutex);
        MsgConfigureSatelliteTrackerWorker& cfg = (MsgConfigureSatelliteTrackerWorker&) message;

        applySettings(cfg.getSettings(), cfg.getSettingsKeys(), cfg.getForce());
        return true;
    }
    else if (SatelliteTracker::MsgSatData::match(message))
    {
        SatelliteTracker::MsgSatData& satData = (SatelliteTracker::MsgSatData&) message;
        m_satellites = satData.getSatellites();
        m_recalculatePasses = true;
        return true;
    }
    else
    {
        return false;
    }
}

void SatelliteTrackerWorker::applySettings(const SatelliteTrackerSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "SatelliteTrackerWorker::applySettings:" << settings.getDebugString(settingsKeys, force) << " force: " << force;

    if (settingsKeys.contains("target")
        || settingsKeys.contains("latitude")
        || settingsKeys.contains("longitude")
        || settingsKeys.contains("heightAboveSeaLevel")
        || settingsKeys.contains("dateTime")
        || settingsKeys.contains("dateTimeSelect")
        || settingsKeys.contains("mapFeature")
        || settingsKeys.contains("fileInputDevice")
        || settingsKeys.contains("cameraFeature")
        || settingsKeys.contains("utc")
        || settingsKeys.contains("groundTrackPoints")
        || settingsKeys.contains("minAOSElevation")
        || settingsKeys.contains("minPassElevation")
        || settingsKeys.contains("predictionPeriod")
        || settingsKeys.contains("passStartTime")
        || settingsKeys.contains("passFinishTime")
        || (!m_settings.m_drawOnMap && settings.m_drawOnMap)
        || force)
    {
        // Recalculate immediately
        m_recalculatePasses = true;
        QTimer::singleShot(1, this, &SatelliteTrackerWorker::update);
        m_pollTimer.start((int)round(settings.m_updatePeriod*1000.0));
    }
    else if (settingsKeys.contains("updatePeriod") || force)
    {
        m_pollTimer.start((int)round(settings.m_updatePeriod*1000.0));
    }

    if (!settings.m_drawOnMap && m_settings.m_drawOnMap)
    {
        QHashIterator<QString, SatWorkerState *> itr(m_workerState);
        while (itr.hasNext())
        {
            itr.next();
            SatWorkerState *satWorkerState = itr.value();
            removeFromMap(itr.key());
            satWorkerState->m_lastSentGroundTrackRevision = 0;
            satWorkerState->m_lastSentPredictedGroundTrackRevision = 0;
        }
    }

    // Remove satellites no longer needed
    QMutableHashIterator<QString, SatWorkerState *> itr(m_workerState);
    while (itr.hasNext())
    {
        itr.next();
        if (settings.m_satellites.indexOf(itr.key()) == -1)
        {
            if (m_settings.m_drawOnMap) {
                removeFromMap(itr.key());
            }
            delete itr.value();
            itr.remove();
        }
    }

    // Add new satellites
    for (int i = 0; i < settings.m_satellites.size(); i++)
    {
        if (!m_workerState.contains(settings.m_satellites[i]))
        {
            SatWorkerState *satWorkerState = new SatWorkerState(settings.m_satellites[i]);
            m_workerState.insert(settings.m_satellites[i], satWorkerState);
            m_recalculatePasses = true;
        }
    }

    if (settingsKeys.contains("target") && (settings.m_target != m_settings.m_target))
    {
        if (m_workerState.contains(m_settings.m_target))
        {
            SatWorkerState *satWorkerState = m_workerState.value(m_settings.m_target);
            disableDoppler(satWorkerState);
        }
        if (m_workerState.contains(settings.m_target))
        {
            SatWorkerState *satWorkerState = m_workerState.value(settings.m_target);
            if (satWorkerState->hasAOS(m_satelliteTracker->currentDateTimeUtc())) {
                enableDoppler(satWorkerState);
            }
        }
    }

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    rescheduleTimer();
}

void SatelliteTrackerWorker::removeFromMap(QString id)
{
    QList<ObjectPipe*> mapMessagePipes;
    MainCore::instance()->getMessagePipes().getMessagePipes(m_satelliteTracker, "mapitems", mapMessagePipes);

    if (mapMessagePipes.size() > 0) {
        sendToMap(mapMessagePipes, id, "", "", "", 0.0f, 0.0, 0.0, 0.0, 0.0);
    }
}

bool SatelliteTrackerWorker::sendToMap(
    const QList<ObjectPipe*>& mapMessagePipes,
    QString name,
    QString image,
    QString model,
    QString text,
    float labelOffset,
    double lat,
    double lon,
    double altitude,
    double rotation,
    const SatelliteTrack *track,
    const SatelliteTrack *predictedTrack
)
{
    bool sent = false;

    for (const auto& pipe : mapMessagePipes)
    {
        MessageQueue *messageQueue = qobject_cast<MessageQueue*>(pipe->m_element);
        if (messageQueue == nullptr) {
            continue;
        }

        SWGSDRangel::SWGMapItem *swgMapItem = new SWGSDRangel::SWGMapItem();
        swgMapItem->setName(new QString(name));
        swgMapItem->setLatitude(lat);
        swgMapItem->setLongitude(lon);
        swgMapItem->setAltitude(altitude);
        swgMapItem->setImage(new QString(image));
        swgMapItem->setImageRotation(rotation);
        swgMapItem->setText(new QString(text));
        swgMapItem->setModel(new QString(model));
        swgMapItem->setFixedPosition(false);
        swgMapItem->setOrientation(0);
        swgMapItem->setLabel(new QString(name));
        swgMapItem->setLabelAltitudeOffset(labelOffset);
        if ((track != nullptr) && track->isValid())
        {
            swgMapItem->setTrackLatitudes(new QList<double>(track->m_latitudes));
            swgMapItem->setTrackLongitudes(new QList<double>(track->m_longitudes));
            swgMapItem->setTrackAltitudes(new QList<double>(track->m_altitudes));
            swgMapItem->setTrackDateTimeMsecs(new QList<qint64>(track->m_dateTimeMsecs));
        }
        if ((predictedTrack != nullptr) && predictedTrack->isValid())
        {
            swgMapItem->setPredictedTrackLatitudes(new QList<double>(predictedTrack->m_latitudes));
            swgMapItem->setPredictedTrackLongitudes(new QList<double>(predictedTrack->m_longitudes));
            swgMapItem->setPredictedTrackAltitudes(new QList<double>(predictedTrack->m_altitudes));
            swgMapItem->setPredictedTrackDateTimeMsecs(new QList<qint64>(predictedTrack->m_dateTimeMsecs));
        }

        MainCore::MsgMapItem *msg = MainCore::MsgMapItem::create(m_satelliteTracker, swgMapItem);
        messageQueue->push(msg);
        sent = true;
    }

    return sent;
}

void SatelliteTrackerWorker::rescheduleTimer()
{
    if (!m_running)
    {
        m_schedulerTimer.stop();
        return;
    }

    const QDateTime now = m_satelliteTracker->currentDateTimeUtc();
    QDateTime nextEvent;

    auto updateNextEvent = [&nextEvent](const QDateTime& dateTime)
    {
        if (dateTime.isValid() && (!nextEvent.isValid() || (dateTime < nextEvent))) {
            nextEvent = dateTime;
        }
    };

    QHashIterator<QString, SatWorkerState *> itr(m_workerState);
    while (itr.hasNext())
    {
        itr.next();
        SatWorkerState *satWorkerState = itr.value();

        if (m_settings.m_dateTimeSelect == SatelliteTrackerSettings::NOW)
        {
            if (satWorkerState->m_aosScheduled) {
                updateNextEvent(satWorkerState->m_aos);
            }

            if (satWorkerState->m_losScheduled) {
                updateNextEvent(satWorkerState->m_los);
            }
        }

        if (satWorkerState->m_dopplerScheduled) {
            updateNextEvent(satWorkerState->m_nextDoppler);
        }
    }

    if (nextEvent.isValid())
    {
        const qint64 msecsToNextEvent = now.msecsTo(nextEvent);
        const int interval = (int) qMin<qint64>(qMax<qint64>(msecsToNextEvent, 1), INT_MAX);
        m_schedulerTimer.start(interval);
    }
    else
    {
        m_schedulerTimer.stop();
    }
}

void SatelliteTrackerWorker::schedulerTick()
{
    const QDateTime now = m_satelliteTracker->currentDateTimeUtc();

    QHashIterator<QString, SatWorkerState *> itr(m_workerState);
    while (itr.hasNext())
    {
        itr.next();
        SatWorkerState *satWorkerState = itr.value();

        if (m_settings.m_dateTimeSelect == SatelliteTrackerSettings::NOW)
        {
            if (satWorkerState->m_aosScheduled && satWorkerState->m_aos.isValid() && (satWorkerState->m_aos <= now))
            {
                satWorkerState->m_aosScheduled = false;
                aos(satWorkerState);
            }

            if (satWorkerState->m_losScheduled && satWorkerState->m_los.isValid() && (satWorkerState->m_los <= now))
            {
                satWorkerState->m_losScheduled = false;
                los(satWorkerState);
                satWorkerState->m_hasSignalledAOS = false;
            }
        }

        if (satWorkerState->m_dopplerScheduled && satWorkerState->m_nextDoppler.isValid() && (satWorkerState->m_nextDoppler <= now))
        {
            const int dopplerPeriodMs = qMax(1, (int) round(m_settings.m_dopplerPeriod * 1000.0f));
            doppler(satWorkerState);
            satWorkerState->m_nextDoppler = now.addMSecs(dopplerPeriodMs);
        }
    }

    rescheduleTimer();
}

void SatelliteTrackerWorker::update()
{
    PROFILER_START();

    bool droppedReport = false;
    const int maxQueueSize = 10;
    QList<SatelliteState> reportSatStates;
    reportSatStates.reserve(m_workerState.size());
    struct MapUpdate
    {
        SatWorkerState *m_satWorkerState;
        QString m_name;
        QString m_image;
        QString m_model;
        QString m_text;
        float m_labelOffset;
        double m_latitude;
        double m_longitude;
        double m_altitude;
        double m_rotation;
    };
    QList<MapUpdate> mapUpdates;
    mapUpdates.reserve(m_workerState.size());

    // Get date and time to calculate position at
    QDateTime qdt;
    if (m_settings.m_dateTime == "")
        qdt = m_satelliteTracker->currentDateTimeUtc();
    else if (m_settings.m_utc)
        qdt = QDateTime::fromString(m_settings.m_dateTime, Qt::ISODateWithMs);
    else
        qdt = QDateTime::fromString(m_settings.m_dateTime, Qt::ISODateWithMs).toUTC();

    bool timeReversed = m_lastUpdateDateTime > qdt;

    // Determine if we need to draw on the map, and thus need to calculate ground tracks
    QList<ObjectPipe*> initialMapMessagePipes;
    MainCore::instance()->getMessagePipes().getMessagePipes(m_satelliteTracker, "mapitems", initialMapMessagePipes);
    auto updateMapPipeCache = [this](const QList<ObjectPipe*>& mapMessagePipes)
    {
        QSet<ObjectPipe *> currentMapMessagePipes;
        for (ObjectPipe *pipe : mapMessagePipes) {
            currentMapMessagePipes.insert(pipe);
        }
        if (currentMapMessagePipes != m_lastMapMessagePipes)
        {
            QHashIterator<QString, SatWorkerState *> stateItr(m_workerState);
            while (stateItr.hasNext())
            {
                stateItr.next();
                SatWorkerState *satWorkerState = stateItr.value();
                satWorkerState->m_lastSentGroundTrackRevision = 0;
                satWorkerState->m_lastSentPredictedGroundTrackRevision = 0;
            }
            m_lastMapMessagePipes = currentMapMessagePipes;
        }
    };
    updateMapPipeCache(initialMapMessagePipes);
    bool mapEnabled = m_settings.m_drawOnMap && (initialMapMessagePipes.size() > 0);

    QHashIterator<QString, SatWorkerState *> itr(m_workerState);
    while (itr.hasNext())
    {
        itr.next();
        SatWorkerState *satWorkerState = itr.value();
        QString name = satWorkerState->m_name;
        if (m_satellites.contains(name))
        {
            SatNogsSatellite *sat = m_satellites.value(name);
            if (sat->m_tle != nullptr)
            {
                // Calculate position, AOS/LOS and other details for satellite
                int noOfPasses;
                bool recalcAsPastLOS = (satWorkerState->m_satState.m_passes.size() > 0) && (satWorkerState->m_satState.m_passes[0].m_los < qdt);
                if (m_recalculatePasses || recalcAsPastLOS || timeReversed)
                    noOfPasses = (name == m_settings.m_target) ? 99 : 1;
                else
                    noOfPasses = 0;
                getSatelliteState(qdt, sat->m_tle->m_tle0, sat->m_tle->m_tle1, sat->m_tle->m_tle2,
                                    m_settings.m_latitude, m_settings.m_longitude, m_settings.m_heightAboveSeaLevel/1000.0,
                                    m_settings.m_predictionPeriod, m_settings.m_minAOSElevation, m_settings.m_minPassElevation,
                                    m_settings.m_passStartTime, m_settings.m_passFinishTime, m_settings.m_utc,
                                    noOfPasses,
                                    mapEnabled, m_settings.m_groundTrackPoints, &satWorkerState->m_satState,
                                    &satWorkerState->m_satStateContext);

                // Update AOS/LOS
                if (satWorkerState->m_satState.m_passes.size() > 0)
                {
                    // Only use timers if using real time
                    if (m_settings.m_dateTimeSelect == SatelliteTrackerSettings::NOW)
                    {
                        // Do we have a new pass?
                        if ((satWorkerState->m_aos != satWorkerState->m_satState.m_passes[0].m_aos) || (satWorkerState->m_los != satWorkerState->m_satState.m_passes[0].m_los))
                        {
                            qDebug() << "SatelliteTrackerWorker: Current time: " << qdt.toString(Qt::ISODateWithMs);
                            qDebug() << "SatelliteTrackerWorker: New AOS: " << name << " new: " << satWorkerState->m_satState.m_passes[0].m_aos << " old: " << satWorkerState->m_aos;
                            qDebug() << "SatelliteTrackerWorker: New LOS: " << name << " new: " << satWorkerState->m_satState.m_passes[0].m_los << " old: " << satWorkerState->m_los;

                            const bool losWasScheduled = satWorkerState->m_losScheduled;
                            const qint64 previousLosRemaining = qdt.msecsTo(satWorkerState->m_los);

                            satWorkerState->m_aos = satWorkerState->m_satState.m_passes[0].m_aos;
                            satWorkerState->m_los = satWorkerState->m_satState.m_passes[0].m_los;
                            satWorkerState->m_hasSignalledAOS = false;
                            satWorkerState->m_aosScheduled = false;
                            satWorkerState->m_losScheduled = false;
                            if (satWorkerState->m_aos.isValid())
                            {
                                if (satWorkerState->m_aos > qdt)
                                {
                                    satWorkerState->m_aosScheduled = true;
                                }
                                else if (qdt < satWorkerState->m_los)
                                    aos(satWorkerState);

                                if (satWorkerState->m_los.isValid() && (m_settings.m_target == satWorkerState->m_name))
                                    calculateRotation(satWorkerState);
                            }
                            if (satWorkerState->m_los.isValid() && (satWorkerState->m_los > qdt))
                            {
                                if (losWasScheduled) {
                                    qDebug() << "SatelliteTrackerWorker::update m_los remaining time: " << previousLosRemaining;
                                }
                                // We can detect a new AOS for a satellite, a little bit before the LOS has occurred
                                // Allow for 5s here (1s doesn't appear to be enough in some cases)
                                if (losWasScheduled && (previousLosRemaining <= 5000))
                                {
                                    // LOS hasn't been called yet - do so, before we reset timer
                                    los(satWorkerState);
                                }
                                qDebug() << "SatelliteTrackerWorker:: Interval to LOS " << (satWorkerState->m_los.toMSecsSinceEpoch() - qdt.toMSecsSinceEpoch());
                                satWorkerState->m_losScheduled = true;
                            }
                        }
                    }
                    else
                    {
                        // Do we need to signal LOS?
                        if (satWorkerState->m_hasSignalledAOS && !satWorkerState->hasAOS(qdt))
                        {
                            los(satWorkerState);
                            satWorkerState->m_hasSignalledAOS = false;
                        }
                        // Do we have a new pass?
                        if ((satWorkerState->m_aos != satWorkerState->m_satState.m_passes[0].m_aos) || (satWorkerState->m_los != satWorkerState->m_satState.m_passes[0].m_los))
                        {
                            satWorkerState->m_aos = satWorkerState->m_satState.m_passes[0].m_aos;
                            satWorkerState->m_los = satWorkerState->m_satState.m_passes[0].m_los;
                            satWorkerState->m_hasSignalledAOS = false;
                        }
                        // Check if we need to signal AOS
                        if (!satWorkerState->m_hasSignalledAOS && satWorkerState->m_aos.isValid() && satWorkerState->hasAOS(qdt)) {
                            aos(satWorkerState);
                        }
                    }
                }
                else
                {
                    satWorkerState->m_aos = QDateTime();
                    satWorkerState->m_los = QDateTime();
                    satWorkerState->m_aosScheduled = false;
                    satWorkerState->m_losScheduled = false;
                }

                // Send Az/El of target to Rotator Controllers, if elevation above horizon
                if ((name == m_settings.m_target) && (satWorkerState->m_satState.m_elevation >= 0))
                {
                    double azimuth = satWorkerState->m_satState.m_azimuth + m_settings.m_azimuthOffset;
                    double elevation = satWorkerState->m_satState.m_elevation + m_settings.m_elevationOffset;
                    if (m_extendedAzRotation)
                    {
                        if (azimuth < 180.0)
                            azimuth += 360.0;
                    }
                    else if (m_flipRotation)
                    {
                        azimuth = std::fmod(azimuth + 180.0, 360.0);
                        elevation = 180.0 - elevation;
                    }

                    QList<ObjectPipe*> rotatorPipes;
                    MainCore::instance()->getMessagePipes().getMessagePipes(m_satelliteTracker, "target", rotatorPipes);

                    for (const auto& pipe : rotatorPipes)
                    {
                        MessageQueue *messageQueue = qobject_cast<MessageQueue*>(pipe->m_element);
                        SWGSDRangel::SWGTargetAzimuthElevation *swgTarget = new SWGSDRangel::SWGTargetAzimuthElevation();
                        swgTarget->setName(new QString(m_settings.m_target));
                        swgTarget->setAzimuth(azimuth);
                        swgTarget->setElevation(elevation);
                        messageQueue->push(MainCore::MsgTargetAzimuthElevation::create(m_satelliteTracker, swgTarget));
                    }
                }

                // Prepare Map update. We re-fetch map pipes once after this compute loop
                // so pipe deregistration is handled close to send time without doing a
                // locked pipe-registry lookup for every satellite.
                if (mapEnabled)
                {
                    static const QStringList cubeSats({"AISAT-1", "FOX-1B", "FOX-1C", "FOX-1D", "FOX-1E", "FUNCUBE-1", "NO-84"});
                    QString image;
                    QString model;
                    float labelOffset;

                    if (sat->m_name == "ISS")
                    {
                        image = "qrc:///satellitetracker/satellitetracker/iss-32.png";
                        model = "iss.glb";
                        labelOffset = 15.0f;
                    }
                    else if (cubeSats.contains(sat->m_name))
                    {
                        image = "qrc:///satellitetracker/satellitetracker/cubesat-32.png";
                        model = "cubesat.glb";
                        labelOffset = 0.7f;
                    }
                    else
                    {
                        image = "qrc:///satellitetracker/satellitetracker/satellite-32.png";
                        model = "satellite.glb";
                        labelOffset = 2.5f;
                    }

                    QString text = QString("Name: %1\nAltitude: %2 km\nRange: %3 km\nRange rate: %4 km/s\nSpeed: %5 km/h\nPeriod: %6 mins")
                                           .arg(sat->m_name)
                                           .arg((int)round(satWorkerState->m_satState.m_altitude))
                                           .arg((int)round(satWorkerState->m_satState.m_range))
                                           .arg(satWorkerState->m_satState.m_rangeRate, 0, 'f', 1)
                                           .arg(Units::kmpsToIntegerKPH(satWorkerState->m_satState.m_speed))
                                           .arg((int)round(satWorkerState->m_satState.m_period));
                    if (satWorkerState->m_satState.m_passes.size() > 0)
                    {
                        if ((qdt >= satWorkerState->m_satState.m_passes[0].m_aos) && (qdt <= satWorkerState->m_satState.m_passes[0].m_los))
                            text = text.append("\nSatellite is visible");
                        else
                            text = text.append("\nAOS in: %1 mins").arg((int)round((satWorkerState->m_satState.m_passes[0].m_aos.toSecsSinceEpoch() - qdt.toSecsSinceEpoch())/60.0));
                        QString aosDateTime;
                        QString losDateTime;
                        if (m_settings.m_utc)
                        {
                            aosDateTime = satWorkerState->m_satState.m_passes[0].m_aos.toString(m_settings.m_dateFormat + " hh:mm");
                            losDateTime = satWorkerState->m_satState.m_passes[0].m_los.toString(m_settings.m_dateFormat + " hh:mm");
                        }
                        else
                        {
                            aosDateTime = satWorkerState->m_satState.m_passes[0].m_aos.toLocalTime().toString(m_settings.m_dateFormat + " hh:mm");
                            losDateTime = satWorkerState->m_satState.m_passes[0].m_los.toLocalTime().toString(m_settings.m_dateFormat + " hh:mm");
                        }
                        text = QString("%1\nAOS: %2\nLOS: %3\nMax El: %4%5")
                                        .arg(text)
                                        .arg(aosDateTime)
                                        .arg(losDateTime)
                                        .arg((int)round(satWorkerState->m_satState.m_passes[0].m_maxElevation))
                                        .arg(QChar(0xb0));
                    }

                    mapUpdates.append(MapUpdate{
                        satWorkerState,
                        sat->m_name,
                        image,
                        model,
                        text,
                        labelOffset,
                        satWorkerState->m_satState.m_latitude,
                        satWorkerState->m_satState.m_longitude,
                        satWorkerState->m_satState.m_altitude * 1000.0,
                        0.0
                    });
                }

                reportSatStates.append(createSatelliteReportState(satWorkerState->m_satState));
            }
            else
                qDebug() << "SatelliteTrackerWorker::update: No TLE for " << sat->m_name << ". Can't compute position.";
        }
    }

    if (!mapUpdates.isEmpty())
    {
        QList<ObjectPipe*> mapMessagePipes;
        MainCore::instance()->getMessagePipes().getMessagePipes(m_satelliteTracker, "mapitems", mapMessagePipes);
        updateMapPipeCache(mapMessagePipes);

        if (mapMessagePipes.size() > 0)
        {
            for (const MapUpdate& mapUpdate : mapUpdates)
            {
                SatWorkerState *satWorkerState = mapUpdate.m_satWorkerState;

                const SatelliteTrack *groundTrack = nullptr;
                if (satWorkerState->m_satState.m_groundTrack.isValid()
                    && (satWorkerState->m_satState.m_groundTrack.m_revision != satWorkerState->m_lastSentGroundTrackRevision))
                {
                    groundTrack = &satWorkerState->m_satState.m_groundTrack;
                }

                const SatelliteTrack *predictedGroundTrack = nullptr;
                if (satWorkerState->m_satState.m_predictedGroundTrack.isValid()
                    && (satWorkerState->m_satState.m_predictedGroundTrack.m_revision != satWorkerState->m_lastSentPredictedGroundTrackRevision))
                {
                    predictedGroundTrack = &satWorkerState->m_satState.m_predictedGroundTrack;
                }

                if (sendToMap(
                    mapMessagePipes,
                    mapUpdate.m_name,
                    mapUpdate.m_image,
                    mapUpdate.m_model,
                    mapUpdate.m_text,
                    mapUpdate.m_labelOffset,
                    mapUpdate.m_latitude,
                    mapUpdate.m_longitude,
                    mapUpdate.m_altitude,
                    mapUpdate.m_rotation,
                    groundTrack,
                    predictedGroundTrack
                ))
                {
                    if (groundTrack != nullptr) {
                        satWorkerState->m_lastSentGroundTrackRevision = groundTrack->m_revision;
                    }
                    if (predictedGroundTrack != nullptr) {
                        satWorkerState->m_lastSentPredictedGroundTrackRevision = predictedGroundTrack->m_revision;
                    }
                }
            }
        }
    }

    if (!reportSatStates.isEmpty())
    {
        // Send to GUI
        if (getMessageQueueToGUI())
        {
            if (getMessageQueueToGUI()->size() < maxQueueSize)
            {
                getMessageQueueToGUI()->push(SatelliteTrackerReport::MsgReportSat::create(reportSatStates));
            }
            else
            {
                qDebug() << "SatelliteTrackerWorker::update: GUI message queue full. Dropping reports for " << reportSatStates.size() << " satellites";
                droppedReport = true;
            }
        }

        // Sent to Feature for Web report
        if (m_msgQueueToFeature)
        {
            if (m_msgQueueToFeature->size() < maxQueueSize)
            {
                m_msgQueueToFeature->push(SatelliteTrackerReport::MsgReportSat::create(reportSatStates));
            }
            else
            {
                qDebug() << "SatelliteTrackerWorker::update: Feature message queue full. Dropping reports for " << reportSatStates.size() << " satellites";
                droppedReport = true;
            }
        }
    }

    m_lastUpdateDateTime = qdt;
    m_recalculatePasses = false;
    rescheduleTimer();

    // If we dropped a report because GUI/feature queues were fairly full, double update period to stop from overloading them
    if (droppedReport)
    {
        m_settings.m_updatePeriod *= 2.0f;
        const int guiQueueSize = getMessageQueueToGUI() ? getMessageQueueToGUI()->size() : -1;
        const int featureQueueSize = m_msgQueueToFeature ? m_msgQueueToFeature->size() : -1;
        qDebug() << "SatelliteTrackerWorker::update: Dropped report. Doubling update period:" << m_settings.m_updatePeriod << "s"
            << "GUI Queue size:" << guiQueueSize
            << "Feature queue size:" << featureQueueSize
            << "maxQueueSize:" << maxQueueSize;
        m_pollTimer.setInterval((int)round(m_settings.m_updatePeriod * 1000.0));
    }

    PROFILER_STOP(__FUNCTION__);
}

void SatelliteTrackerWorker::aos(SatWorkerState *satWorkerState)
{
    qDebug() << "SatelliteTrackerWorker::aos " << satWorkerState->m_name;

    satWorkerState->m_hasSignalledAOS = true;

    // Indicate AOS to GUI
    if (getMessageQueueToGUI())
    {
        QString speech = substituteVariables(m_settings.m_aosSpeech, satWorkerState->m_name);
        getMessageQueueToGUI()->push(SatelliteTrackerReport::MsgReportAOS::create(satWorkerState->m_name, speech));
    }

    // Update target
    if (m_settings.m_autoTarget && (satWorkerState->m_name != m_settings.m_target))
    {
        // Only switch if higher priority (earlier in list) or other target not in AOS
        SatWorkerState *targetSatWorkerState = m_workerState.value(m_settings.m_target);
        int currentTargetIdx = m_settings.m_satellites.indexOf(m_settings.m_target);
        int newTargetIdx = m_settings.m_satellites.indexOf(satWorkerState->m_name);
        if ((newTargetIdx < currentTargetIdx) || !targetSatWorkerState->hasAOS(m_satelliteTracker->currentDateTimeUtc()))
        {
            // Stop doppler correction for current target
            if (m_workerState.contains(m_settings.m_target))
                disableDoppler(m_workerState.value(m_settings.m_target));

            qDebug() << "SatelliteTrackerWorker::aos - autoTarget setting " << satWorkerState->m_name;
            m_settings.m_target = satWorkerState->m_name;
            // Update GUI with new target
            if (getMessageQueueToGUI())
                getMessageQueueToGUI()->push(SatelliteTrackerReport::MsgReportTarget::create(satWorkerState->m_name));
        }
    }

    // TODO: Detect if different device sets are used and support multiple sats simultaneously
    if (m_settings.m_target == satWorkerState->m_name)
        applyDeviceAOSSettings(satWorkerState->m_name);

    // Send event to other features
    sendEvent(satWorkerState, true);
}

// Determine if we need to flip rotator or use extended azimuth to avoid 360/0 discontinuity
void SatelliteTrackerWorker::calculateRotation(SatWorkerState *satWorkerState)
{
    m_flipRotation = false;
    m_extendedAzRotation = false;
    if (satWorkerState->m_satState.m_passes.size() > 0)
    {
        SatNogsSatellite *sat = m_satellites.value(satWorkerState->m_name);
        bool passes0 = getPassesThrough0Deg(sat->m_tle->m_tle0, sat->m_tle->m_tle1, sat->m_tle->m_tle2,
                                            m_settings.m_latitude, m_settings.m_longitude, m_settings.m_heightAboveSeaLevel/1000.0,
                                            satWorkerState->m_satState.m_passes[0].m_aos, satWorkerState->m_satState.m_passes[0].m_los);
        if (passes0)
        {
            double aosAz = satWorkerState->m_satState.m_passes[0].m_aosAzimuth;
            double losAz = satWorkerState->m_satState.m_passes[0].m_losAzimuth;
            double minAz = std::min(aosAz, losAz);
            if ((m_settings.m_rotatorMaxAzimuth - 360.0) > minAz)
                m_extendedAzRotation = true;
            else if (m_settings.m_rotatorMaxElevation == 180.0)
                m_flipRotation = true;
        }
    }
}

QString SatelliteTrackerWorker::substituteVariables(const QString &textIn, const QString &satelliteName)
{
    SatWorkerState *satWorkerState = m_workerState.value(satelliteName);
    if (!satWorkerState) {
        return "";
    }

    int durationMins = (int)round((satWorkerState->m_los.toSecsSinceEpoch() - satWorkerState->m_aos.toSecsSinceEpoch())/60.0);

    QString text = textIn;
    text = text.replace("${name}", satelliteName);
    text = text.replace("${duration}", QString::number(durationMins));
    if (satWorkerState->m_satState.m_passes.size() > 0)
    {
        text = text.replace("${aos}", satWorkerState->m_satState.m_passes[0].m_aos.toString());
        text = text.replace("${los}", satWorkerState->m_satState.m_passes[0].m_los.toString());
        text = text.replace("${elevation}", QString::number(std::round(satWorkerState->m_satState.m_passes[0].m_maxElevation)));
        text = text.replace("${aosAzimuth}", QString::number(std::round(satWorkerState->m_satState.m_passes[0].m_aosAzimuth)));
        text = text.replace("${losAzimuth}", QString::number(std::round(satWorkerState->m_satState.m_passes[0].m_losAzimuth)));
        text = text.replace("${northToSouth}", QString::number(satWorkerState->m_satState.m_passes[0].m_northToSouth));
        text = text.replace("${latitude}", QString::number(satWorkerState->m_satState.m_latitude));
        text = text.replace("${longitude}", QString::number(satWorkerState->m_satState.m_longitude));
        text = text.replace("${altitude}", QString::number(satWorkerState->m_satState.m_altitude));
        text = text.replace("${azimuth}", QString::number(std::round(satWorkerState->m_satState.m_azimuth)));
        text = text.replace("${elevation}", QString::number(std::round(satWorkerState->m_satState.m_elevation)));
        text = text.replace("${range}", QString::number(std::round(satWorkerState->m_satState.m_range)));
        text = text.replace("${rangeRate}", QString::number(std::round(satWorkerState->m_satState.m_rangeRate)));
        text = text.replace("${speed}", QString::number(std::round(satWorkerState->m_satState.m_speed)));
        text = text.replace("${period}", QString::number(satWorkerState->m_satState.m_period));
    }
    return text;
}

void SatelliteTrackerWorker::executeCommand(const QString &command, const QString &satelliteName)
{
    if (!command.isEmpty())
    {
#if QT_CONFIG(process)
        // Replace variables
        QString cmd = substituteVariables(command, satelliteName);
        QStringList allArgs = QProcess::splitCommand(cmd);
        qDebug() << "SatelliteTrackerWorker::executeCommand: Executing: " << allArgs;
        QString program = allArgs[0];
        allArgs.pop_front();
        QProcess::startDetached(program, allArgs);
#else
        qWarning() << "SatelliteTrackerWorker::executeCommand: QProcess not supported. Can't run: " << command;
#endif
    }
}

void SatelliteTrackerWorker::applyDeviceAOSSettings(const QString& name)
{
    // Execute global program/script
    if (!m_settings.m_aosCommand.isEmpty()) {
        executeCommand(m_settings.m_aosCommand, name);
    }

    // Update device set
    if (m_settings.m_deviceSettings.contains(name))
    {
        QList<SatelliteTrackerSettings::SatelliteDeviceSettings *> *m_deviceSettingsList = m_settings.m_deviceSettings.value(name);

        MainCore *mainCore = MainCore::instance();

        // Load presets
        for (int i = 0; i < m_deviceSettingsList->size(); i++)
        {
            SatelliteTrackerSettings::SatelliteDeviceSettings *devSettings = m_deviceSettingsList->at(i);
            if (!devSettings->m_presetGroup.isEmpty())
            {
                const MainSettings& mainSettings = mainCore->getSettings();
                const std::vector<DeviceSet*>& deviceSets = mainCore->getDeviceSets();

                if (devSettings->m_deviceSetIndex < (int)deviceSets.size())
                {
                    const DeviceSet *deviceSet = deviceSets[devSettings->m_deviceSetIndex];
                    QString presetType;
                    if (deviceSet->m_deviceSourceEngine != nullptr) {
                        presetType = "R";
                    } else if (deviceSet->m_deviceSinkEngine != nullptr) {
                        presetType = "T";
                    } else if (deviceSet->m_deviceMIMOEngine != nullptr) {
                        presetType = "M";
                    }

                    const Preset* preset = mainSettings.getPreset(devSettings->m_presetGroup, devSettings->m_presetFrequency, devSettings->m_presetDescription, presetType);

                    if (preset != nullptr)
                    {
                        qDebug() << "SatelliteTrackerWorker::aos: Loading preset " << preset->getDescription() << " to device set at " << devSettings->m_deviceSetIndex;
                        MainCore::MsgLoadPreset *msg = MainCore::MsgLoadPreset::create(preset, devSettings->m_deviceSetIndex);
                        mainCore->getMainMessageQueue()->push(msg);
                    }
                    else
                    {
                        qWarning() << "SatelliteTrackerWorker::aos: Unable to get preset: " << devSettings->m_presetGroup << " " << devSettings->m_presetFrequency << " " << devSettings->m_presetDescription;
                    }
                }
                else
                {
                    qWarning() << "SatelliteTrackerWorker::aos: device set at " << devSettings->m_deviceSetIndex << " does not exist";
                }
            }
        }

        // Wait a little bit for presets to load before performing other steps
        QTimer::singleShot(1000, [this, name, m_deviceSettingsList]()
        {

            for (int i = 0; i < m_deviceSettingsList->size(); i++)
            {
                SatelliteTrackerSettings::SatelliteDeviceSettings *devSettings = m_deviceSettingsList->at(i);

                // Override frequency
                if (devSettings->m_frequency != 0)
                {
                    qDebug() << "SatelliteTrackerWorker::aos: setting frequency to: " << devSettings->m_frequency;
                    ChannelWebAPIUtils::setCenterFrequency(devSettings->m_deviceSetIndex, devSettings->m_frequency);
                }

                // Execute per satellite program/script
                if (!devSettings->m_aosCommand.isEmpty()) {
                    executeCommand(devSettings->m_aosCommand, name);
                }

            }

            // Start acquisition - Need to use WebAPI, in order for GUI to correctly reflect being started
            for (int i = 0; i < m_deviceSettingsList->size(); i++)
            {
                SatelliteTrackerSettings::SatelliteDeviceSettings *devSettings = m_deviceSettingsList->at(i);
                if (devSettings->m_startOnAOS)
                {
                    qDebug() << "SatelliteTrackerWorker::aos: starting acquisition";
                    ChannelWebAPIUtils::run(devSettings->m_deviceSetIndex);
                }
            }

            // Send AOS message to channels/features
            SatWorkerState *satWorkerState = m_workerState.value(name);
            SatNogsSatellite *sat = m_satellites.value(satWorkerState->m_name);
            // APT needs current time, for current position of satellite, not start of pass which may be in the past
            // if the satellite was already visible when Sat Tracker was started
            ChannelWebAPIUtils::satelliteAOS(name, satWorkerState->m_satState.m_passes[0].m_northToSouth,
                                             sat->m_tle->toString(),
                                             m_satelliteTracker->currentDateTimeUtc());
            FeatureWebAPIUtils::satelliteAOS(name, satWorkerState->m_aos, satWorkerState->m_los);

            // Start Doppler correction, if needed
            enableDoppler(satWorkerState);

            // Start file sinks (need a little delay to ensure sample rate message has been handled in filerecord)
            QTimer::singleShot(1000, [m_deviceSettingsList]()
            {
                for (int i = 0; i < m_deviceSettingsList->size(); i++)
                {
                    SatelliteTrackerSettings::SatelliteDeviceSettings *devSettings = m_deviceSettingsList->at(i);

                    if (devSettings->m_startStopFileSink)
                    {
                        qDebug() << "SatelliteTrackerWorker::aos: starting file sinks";
                        ChannelWebAPIUtils::startStopFileSinks(devSettings->m_deviceSetIndex, true);
                    }
                }
            });

        });
    }
    else
    {
        // Send AOS message to channels/features
        SatWorkerState *satWorkerState = m_workerState.value(name);
        SatNogsSatellite *sat = m_satellites.value(satWorkerState->m_name);
        ChannelWebAPIUtils::satelliteAOS(name, satWorkerState->m_satState.m_passes[0].m_northToSouth,
                                            sat->m_tle->toString(),
                                            m_satelliteTracker->currentDateTimeUtc());
        FeatureWebAPIUtils::satelliteAOS(name, satWorkerState->m_aos, satWorkerState->m_los);
    }

}

void SatelliteTrackerWorker::enableDoppler(SatWorkerState *satWorkerState)
{
    QList<SatelliteTrackerSettings::SatelliteDeviceSettings *> *m_deviceSettingsList = m_settings.m_deviceSettings.value(satWorkerState->m_name);
    if (m_deviceSettingsList)
    {
        satWorkerState->m_doppler.clear();
        bool requiresDoppler = false;
        for (int i = 0; i < m_deviceSettingsList->size(); i++)
        {
            SatelliteTrackerSettings::SatelliteDeviceSettings *devSettings = m_deviceSettingsList->at(i);
            if (devSettings->m_doppler.size() > 0)
            {
                requiresDoppler = true;
                satWorkerState->m_doppler.append(0);
            }
        }
        if (requiresDoppler)
        {
            qDebug() << "SatelliteTrackerWorker::applyDeviceAOSSettings: Enabling doppler for " << satWorkerState->m_name;
            const int dopplerPeriodMs = qMax(1, (int) round(m_settings.m_dopplerPeriod * 1000.0f));
            satWorkerState->m_nextDoppler = m_satelliteTracker->currentDateTimeUtc().addMSecs(dopplerPeriodMs);
            satWorkerState->m_dopplerScheduled = true;
            rescheduleTimer();
        }
        else
        {
            satWorkerState->m_nextDoppler = QDateTime();
            satWorkerState->m_dopplerScheduled = false;
            rescheduleTimer();
        }
    }
}

void SatelliteTrackerWorker::disableDoppler(SatWorkerState *satWorkerState)
{
    satWorkerState->m_nextDoppler = QDateTime();
    satWorkerState->m_dopplerScheduled = false;
    // Remove doppler correction from any channel
    if (satWorkerState->m_doppler.size() > 0)
    {
        QList<SatelliteTrackerSettings::SatelliteDeviceSettings *> *m_deviceSettingsList = m_settings.m_deviceSettings.value(satWorkerState->m_name);
        if (m_deviceSettingsList)
        {
            int idx = 0;

            for (int i = 0; i < m_deviceSettingsList->size(); i++)
            {
                SatelliteTrackerSettings::SatelliteDeviceSettings *devSettings = m_deviceSettingsList->at(i);
                if (devSettings->m_doppler.size() > 0)
                {
                    for (int j = 0; j < devSettings->m_doppler.size(); j++)
                    {
                        int offset;
                        if (ChannelWebAPIUtils::getFrequencyOffset(devSettings->m_deviceSetIndex, devSettings->m_doppler[j], offset))
                        {
                            // Remove old doppler
                            std::vector<DeviceSet*>& deviceSets = MainCore::instance()->getDeviceSets();
                            ChannelAPI *channel = deviceSets[devSettings->m_deviceSetIndex]->getChannelAt(j);
                            int tx = false;
                            if (channel) {
                                tx = channel->getStreamType() == ChannelAPI::StreamSingleSource; // What if MIMO?
                            }
                            if (tx) {
                                offset -= satWorkerState->m_doppler[idx];
                            } else {
                                offset += satWorkerState->m_doppler[idx];
                            }
                            if (!ChannelWebAPIUtils::setFrequencyOffset(devSettings->m_deviceSetIndex, devSettings->m_doppler[j], offset))
                                qDebug() << "SatelliteTrackerWorker::disableDoppler: Failed to set frequency offset";
                        }
                        else
                            qDebug() << "SatelliteTrackerWorker::disableDoppler: Failed to get frequency offset";
                    }
                    satWorkerState->m_doppler[idx] = 0;
                    idx++;
                }
            }
        }
    }
    rescheduleTimer();
}

void SatelliteTrackerWorker::doppler(SatWorkerState *satWorkerState)
{
    qDebug() << "SatelliteTrackerWorker::doppler " << satWorkerState->m_name;

    QList<SatelliteTrackerSettings::SatelliteDeviceSettings *> *m_deviceSettingsList = m_settings.m_deviceSettings.value(satWorkerState->m_name);
    if (m_deviceSettingsList)
    {
        int idx = 0;

        for (int i = 0; i < m_deviceSettingsList->size(); i++)
        {
            SatelliteTrackerSettings::SatelliteDeviceSettings *devSettings = m_deviceSettingsList->at(i);
            if (devSettings->m_doppler.size() > 0)
            {
                // Get center frequency for this device
                double centerFrequency;

                if (ChannelWebAPIUtils::getCenterFrequency(devSettings->m_deviceSetIndex, centerFrequency))
                {
                    // Calculate frequency delta due to Doppler
                    double c = 299792458.0;
                    double deltaF = centerFrequency * satWorkerState->m_satState.m_rangeRate * 1000.0 / c;
                    int doppler = (int)round(deltaF);

                    for (int j = 0; j < devSettings->m_doppler.size(); j++)
                    {
                        int offset;
                        if (ChannelWebAPIUtils::getFrequencyOffset(devSettings->m_deviceSetIndex, devSettings->m_doppler[j], offset))
                        {
                            // Apply doppler - For receive, we subtract, transmit we add
                            std::vector<DeviceSet*>& deviceSets = MainCore::instance()->getDeviceSets();
                            ChannelAPI *channel = deviceSets[devSettings->m_deviceSetIndex]->getChannelAt(j);
                            int tx = false;
                            if (channel) {
                                tx = channel->getStreamType() == ChannelAPI::StreamSingleSource; // What if MIMO?
                            }

                            // Remove old doppler and apply new
                            int initOffset;
                            if (tx)
                            {
                                initOffset = offset - satWorkerState->m_doppler[idx];
                                offset = initOffset + doppler;
                            }
                            else
                            {
                                initOffset = offset + satWorkerState->m_doppler[idx];
                                offset = initOffset - doppler;
                            }
                            if (!ChannelWebAPIUtils::setFrequencyOffset(devSettings->m_deviceSetIndex, devSettings->m_doppler[j], offset))
                                qDebug() << "SatelliteTrackerWorker::doppler: Failed to set frequency offset";
                        }
                        else
                            qDebug() << "SatelliteTrackerWorker::doppler: Failed to get frequency offset";
                    }

                    satWorkerState->m_doppler[idx] = doppler;
                }
                else
                    qDebug() << "SatelliteTrackerWorker::doppler: couldn't get centre frequency for device at " << devSettings->m_deviceSetIndex;

                idx++;
            }
        }
    }
}

void SatelliteTrackerWorker::los(SatWorkerState *satWorkerState)
{
    qDebug() << "SatelliteTrackerWorker::los " << satWorkerState->m_name << " target: " << m_settings.m_target;

    // Indicate LOS to GUI
    if (getMessageQueueToGUI())
    {
        QString speech = substituteVariables(m_settings.m_losSpeech, satWorkerState->m_name);
        getMessageQueueToGUI()->push(SatelliteTrackerReport::MsgReportLOS::create(satWorkerState->m_name, speech));
    }

    disableDoppler(satWorkerState);

    if (m_settings.m_target == satWorkerState->m_name)
    {
        // Execute program/script
        if (!m_settings.m_losCommand.isEmpty()) {
            executeCommand(m_settings.m_losCommand, satWorkerState->m_name);
        }

        // Send LOS message to channels/features
        ChannelWebAPIUtils::satelliteLOS(satWorkerState->m_name);
        FeatureWebAPIUtils::satelliteLOS(satWorkerState->m_name);

        // Send event to other features
        sendEvent(satWorkerState, false);

        if (m_settings.m_deviceSettings.contains(satWorkerState->m_name))
        {
            QList<SatelliteTrackerSettings::SatelliteDeviceSettings *> *m_deviceSettingsList = m_settings.m_deviceSettings.value(satWorkerState->m_name);

            // Stop file sinks
            for (int i = 0; i < m_deviceSettingsList->size(); i++)
            {
                SatelliteTrackerSettings::SatelliteDeviceSettings *devSettings = m_deviceSettingsList->at(i);
                if (devSettings->m_startStopFileSink)
                {
                    qDebug() << "SatelliteTrackerWorker::los: stopping file sinks";
                    ChannelWebAPIUtils::startStopFileSinks(devSettings->m_deviceSetIndex, false);
                }
            }

            // Stop acquisition
            for (int i = 0; i < m_deviceSettingsList->size(); i++)
            {
                SatelliteTrackerSettings::SatelliteDeviceSettings *devSettings = m_deviceSettingsList->at(i);

                if (devSettings->m_stopOnLOS) {
                    ChannelWebAPIUtils::stop(devSettings->m_deviceSetIndex);
                }
            }

            // Execute per satellite program/script
            // Do after stopping acquisition, so files are closed by file sink
            for (int i = 0; i < m_deviceSettingsList->size(); i++)
            {
                SatelliteTrackerSettings::SatelliteDeviceSettings *devSettings = m_deviceSettingsList->at(i);
                if (!devSettings->m_losCommand.isEmpty()) {
                    executeCommand(devSettings->m_losCommand, satWorkerState->m_name);
                }
            }
        }
    }

    // Is another lower-priority satellite with AOS available to switch to?
    if (m_settings.m_autoTarget)
    {
        for (int i = m_settings.m_satellites.indexOf(m_settings.m_target) + 1; i < m_settings.m_satellites.size(); i++)
        {
            if (m_workerState.contains(m_settings.m_satellites[i]))
            {
                SatWorkerState *newSatWorkerState = m_workerState.value(m_settings.m_satellites[i]);
                if (newSatWorkerState->hasAOS(m_satelliteTracker->currentDateTimeUtc()))
                {
                    qDebug() << "SatelliteTrackerWorker::los - autoTarget setting " << m_settings.m_satellites[i];
                    m_settings.m_target = m_settings.m_satellites[i];
                    // Update GUI with new target
                    if (getMessageQueueToGUI())
                        getMessageQueueToGUI()->push(SatelliteTrackerReport::MsgReportTarget::create(m_settings.m_target));
                    // Apply device settings
                    applyDeviceAOSSettings(m_settings.m_target);
                    break;
                }
            }
        }
    }

    m_recalculatePasses = true;
}

void SatelliteTrackerWorker::sendEvent(const SatWorkerState *satWorkerState, bool aos)
{
    QList<ObjectPipe*> eventPipes;
    MainCore::instance()->getMessagePipes().getMessagePipes(m_satelliteTracker, "event", eventPipes);
    QString eventData = QString("name=%1").arg(satWorkerState->m_name);
    MainCore::MsgEvent::EventType eventType = aos ? MainCore::MsgEvent::EventType::SatelliteAOSEvent : MainCore::MsgEvent::SatelliteLOSEvent;
    QDateTime eventTime = aos ? satWorkerState->m_aos : satWorkerState->m_los;
    for (const auto& pipe : eventPipes)
    {
        MessageQueue *messageQueue = qobject_cast<MessageQueue*>(pipe->m_element);
        messageQueue->push(MainCore::MsgEvent::create(m_satelliteTracker, eventTime, eventType, eventData));
    }
}

bool SatWorkerState::hasAOS(const QDateTime& currentTime)
{
    return (m_aos <= currentTime) && (m_los > currentTime);
}
