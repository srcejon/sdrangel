#ifndef INCLUDE_FEATURE_CAMERAINFO_H_
#define INCLUDE_FEATURE_CAMERAINFO_H_

#include <QList>
#include <QString>

struct CameraInfo
{
    QString m_protocol;     // "qt" for Qt cameras, "alpaca" for Alpaca cameras
    QString m_id;
    QString m_description;
    QString m_host;         // alpaca only
    quint16 m_port = 0;     // alpaca only
};

struct AlpacaDeviceInfo
{
    QString m_type;         // e.g. "camera", "focuser", "filterwheel"
    QString m_id;           // A number, typically 0
    QString m_description;
    QString m_host;
    quint16 m_port = 0;
};

#endif // INCLUDE_FEATURE_CAMERAINFO_H_
