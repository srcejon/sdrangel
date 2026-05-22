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

#ifndef INCLUDE_FEATURE_CAMERAPLATESOLVER_H_
#define INCLUDE_FEATURE_CAMERAPLATESOLVER_H_

#include <QByteArray>
#include <QDateTime>
#include <QHash>
#include <QObject>
#include <QSize>
#include <QString>
#include <QVector>

#include "camerapipelineframe.h"
#include "camerasettings.h"

struct CameraPlateSolveResult
{
    bool m_solved = false;
    int m_matchedStars = 0;
    int m_detectedStarsConsidered = 0;
    int m_catalogStarsLoaded = 0;
    int m_catalogCandidateStars = 0;
    int m_outlierStars = 0;
    double m_rmsErrorPixels = 0.0;
    double m_maxErrorPixels = 0.0;
    double m_azimuthDegrees = 0.0;
    double m_elevationDegrees = 0.0;
    double m_rollDegrees = 0.0;
    double m_fovDegrees = 0.0;
    double m_centerOffsetXPixels = 0.0;
    double m_centerOffsetYPixels = 0.0;
    double m_distortionK1 = 0.0;
    QString m_catalogSource;
    QString m_failureReason;
    QString m_matchSummary;
    int m_requiredMatches = 0;
};

class QNetworkAccessManager;
class QNetworkReply;

class CameraPlateSolver : public QObject
{
public:
    explicit CameraPlateSolver(QObject *parent = nullptr);
    ~CameraPlateSolver() override;

    static QString downloadedCatalogArchivePath();
    static QString downloadedCatalogCsvPath();
    static bool importDownloadedCatalogArchive(const QString& archivePath, QString* errorMessage = nullptr);
    CameraPlateSolveResult solve(const CameraSettings& settings,
                                 const QSize& imageSize,
                                 const QDateTime& captureDateTime,
                                 QVector<CameraPipelineStarDetection>& starDetections);
    // Cancel any Siril SPCC network request currently blocking in loop.exec() inside
    // fetchSirilRangeFromSource.  Safe to call from the star-detector thread during
    // event processing (e.g. from CameraStarDetector::captureActiveChanged(false)).
    void requestNetworkCancellation();

private:
    class SolverContext;
    friend class SolverContext;
    QNetworkAccessManager *m_networkManager = nullptr;
    // Siril SPCC catalog caches — persisted across solve() calls so repeated plate
    // solves on the same sky area don't re-download the same data each frame.
    QHash<QString, QByteArray> m_sirilRangeCache;
    QHash<int, QByteArray> m_sirilIndexCache;
    // Cancellation support — both members accessed only on the star-detector thread.
    bool m_cancelNetworkRequests = false;
    QNetworkReply *m_activeNetworkReply = nullptr;
};

#endif // INCLUDE_FEATURE_CAMERAPLATESOLVER_H_
