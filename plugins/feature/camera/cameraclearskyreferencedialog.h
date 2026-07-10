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
// GNU General Public License V3 for more details.                              //
//                                                                               //
// You should have received a copy of the GNU General Public License             //
// along with this program. If not, see <http://www.gnu.org/licenses/>.          //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_FEATURE_CAMERACLEARSKYREFERENCEDIALOG_H_
#define INCLUDE_FEATURE_CAMERACLEARSKYREFERENCEDIALOG_H_

#include <QDialog>

#include <functional>

class QGridLayout;

/**
 * \brief Viewer for the per-camera clear-sky reference store.
 *
 * Shows, for each sky-state slot, the stored maps (reconstructed clear-sky brightness,
 * red/blue colour ratio, fine texture, evaluated-sky mask) together with capture metadata,
 * plus the foreground mask derived from them, so the user can check that saved and
 * auto-learned references look sensible. Reads the store directly from disk (saves are
 * atomic), so it always reflects the persisted state; Refresh re-reads it.
 */
class CameraClearSkyReferenceDialog : public QDialog
{
    Q_OBJECT
public:
    // clearRequested is invoked when the user confirms deleting this camera's reference
    // store; the owner routes it to the running detector so the in-memory store clears too
    explicit CameraClearSkyReferenceDialog(const QString& storageKey, std::function<void()> clearRequested, QWidget *parent = nullptr);

private slots:
    void refresh();
    void clearReferences();

private:
    QString m_storageKey;
    std::function<void()> m_clearRequested;
    QGridLayout *m_grid;
};

#endif // INCLUDE_FEATURE_CAMERACLEARSKYREFERENCEDIALOG_H_
