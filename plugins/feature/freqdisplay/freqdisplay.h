///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by Copilot                                                          //
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

#ifndef INCLUDE_FEATURE_FREQDISPLAY_H_
#define INCLUDE_FEATURE_FREQDISPLAY_H_

#include <QNetworkRequest>
#include <QStringList>

#include "feature/feature.h"
#include "util/message.h"

#include "freqdisplaysettings.h"

class WebAPIAdapterInterface;
class QNetworkAccessManager;
class QNetworkReply;

namespace SWGSDRangel {
    class SWGFeatureSettings;
    class SWGFeatureReport;
}

class FreqDisplay : public Feature
{
    Q_OBJECT
public:
    class MsgConfigureFreqDisplay : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const FreqDisplaySettings& getSettings() const { return m_settings; }
        const QStringList& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }

        static MsgConfigureFreqDisplay* create(const FreqDisplaySettings& settings, const QStringList& settingsKeys, bool force) {
            return new MsgConfigureFreqDisplay(settings, settingsKeys, force);
        }

    private:
        FreqDisplaySettings m_settings;
        QStringList m_settingsKeys;
        bool m_force;

        MsgConfigureFreqDisplay(const FreqDisplaySettings& settings, const QStringList& settingsKeys, bool force) :
            Message(),
            m_settings(settings),
            m_settingsKeys(settingsKeys),
            m_force(force)
        { }
    };

    FreqDisplay(WebAPIAdapterInterface *webAPIAdapterInterface);
    ~FreqDisplay() override;

    void destroy() override { delete this; }
    bool handleMessage(const Message& cmd) override;

    void getIdentifier(QString& id) const override { id = objectName(); }
    QString getIdentifier() const override { return objectName(); }
    void getTitle(QString& title) const override { title = m_settings.m_title; }

    QByteArray serialize() const override;
    bool deserialize(const QByteArray& data) override;

    int webapiSettingsGet(
        SWGSDRangel::SWGFeatureSettings& response,
        QString& errorMessage) override;

    int webapiSettingsPutPatch(
        bool force,
        const QStringList& featureSettingsKeys,
        SWGSDRangel::SWGFeatureSettings& response,
        QString& errorMessage) override;

    int webapiReportGet(
        SWGSDRangel::SWGFeatureReport& response,
        QString& errorMessage) override;

    static void webapiFormatFeatureSettings(
        SWGSDRangel::SWGFeatureSettings& response,
        const FreqDisplaySettings& settings);

    static void webapiUpdateFeatureSettings(
        FreqDisplaySettings& settings,
        const QStringList& featureSettingsKeys,
        SWGSDRangel::SWGFeatureSettings& response);

    static const char* const m_featureIdURI;
    static const char* const m_featureId;

private:
    FreqDisplaySettings m_settings;
    QNetworkAccessManager *m_networkManager;
    QNetworkRequest m_networkRequest;

    void webapiReverseSendSettings(const QStringList& featureSettingsKeys, const FreqDisplaySettings& settings, bool force);
    void webapiFormatFeatureReport(SWGSDRangel::SWGFeatureReport& response);
    void applySettings(const FreqDisplaySettings& settings, const QStringList& settingsKeys, bool force = false);

private slots:
    void networkManagerFinished(QNetworkReply *reply);
};

#endif // INCLUDE_FEATURE_FREQDISPLAY_H_
