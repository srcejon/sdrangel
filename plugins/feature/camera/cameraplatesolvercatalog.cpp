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

// Catalog star naming / display helpers. Member definitions moved out-of-line from
// SolverContext (WS5 phase 2) to split the former single ~21k-line translation unit.

QString CameraPlateSolver::SolverContext::formatGaiaCoordinateLabel(double rightAscensionDegrees,
                                         double declinationDegrees)
{
    constexpr int tenthsPerHour = 60 * 60 * 10;
    constexpr int tenthsPerDay = 24 * tenthsPerHour;
    int totalTenths = static_cast<int>(std::round(normalizeDegrees(rightAscensionDegrees) / 15.0 * tenthsPerHour));
    if (totalTenths >= tenthsPerDay) {
        totalTenths -= tenthsPerDay;
    }
    const int hours = totalTenths / tenthsPerHour;
    totalTenths %= tenthsPerHour;
    const int minutes = totalTenths / (60 * 10);
    const double seconds = (totalTenths % (60 * 10)) / 10.0;

    const double absoluteDeclinationDegrees = std::fabs(declinationDegrees);
    const int degrees = static_cast<int>(std::floor(absoluteDeclinationDegrees));
    const double totalArcMinutes = (absoluteDeclinationDegrees - degrees) * 60.0;
    const int arcMinutes = static_cast<int>(std::floor(totalArcMinutes));
    int arcSeconds = static_cast<int>(std::round((totalArcMinutes - arcMinutes) * 60.0));
    int normalizedArcMinutes = arcMinutes;
    int normalizedDegrees = degrees;
    if (arcSeconds >= 60)
    {
        arcSeconds = 0;
        ++normalizedArcMinutes;
        if (normalizedArcMinutes >= 60)
        {
            normalizedArcMinutes = 0;
            ++normalizedDegrees;
        }
    }

    const QChar declinationSign = declinationDegrees < 0.0 ? QLatin1Char('-') : QLatin1Char('+');
    return QStringLiteral("Gaia J%1%2%3%4%5%6%7")
        .arg(hours, 2, 10, QLatin1Char('0'))
        .arg(minutes, 2, 10, QLatin1Char('0'))
        .arg(seconds, 4, 'f', 1, QLatin1Char('0'))
        .arg(declinationSign)
        .arg(normalizedDegrees, 2, 10, QLatin1Char('0'))
        .arg(normalizedArcMinutes, 2, 10, QLatin1Char('0'))
        .arg(arcSeconds, 2, 10, QLatin1Char('0'));
}

bool CameraPlateSolver::SolverContext::isGenericGaiaCatalogName(const QString& name)
{
    const QString trimmed = name.trimmed();
    return trimmed.startsWith(QStringLiteral("Gaia Astro "), Qt::CaseInsensitive)
        || trimmed.startsWith(QStringLiteral("Gaia SPCC "), Qt::CaseInsensitive)
        || trimmed.startsWith(QStringLiteral("Gaia J"), Qt::CaseInsensitive);
}

QString CameraPlateSolver::SolverContext::catalogDisplayName(const CatalogStar& star)
{
    // Trust the name already assigned to this star (set when the catalog was loaded/merged,
    // see applyNamedAliasesToCatalog/mergeBundledBrightStarsIntoCatalog -- both perform a
    // one-to-one assignment so a given alias name identifies a single catalog entry).
    //
    // Previously this independently re-resolved the alias for every star on every call,
    // which duplicated applyNamedAliasesToCatalog's matching (with the same many-to-one
    // pitfalls) and could override an already-correct name with a different, merely
    // closer-scoring alias on the fly -- e.g. turning a correctly merged bright star's name
    // into a different nearby alias's name purely because of how display happened to be
    // invoked for it.
    if (isGenericGaiaCatalogName(star.name)) {
        return formatGaiaCoordinateLabel(star.rightAscensionDegrees, star.declinationDegrees);
    }
    return star.name;
}
