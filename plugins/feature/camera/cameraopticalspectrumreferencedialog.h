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

#ifndef INCLUDE_FEATURE_CAMERAOPTICALSPECTRUMREFERENCEDIALOG_H_
#define INCLUDE_FEATURE_CAMERAOPTICALSPECTRUMREFERENCEDIALOG_H_

#include <QDialog>
#include <QString>

class QComboBox;
class QLabel;
class QLineEdit;
class QNetworkAccessManager;
class QNetworkReply;
class QPushButton;

/**
 * \brief Dialog for choosing a reference spectrum to overlay on the optical spectrum chart.
 *
 * A star can be entered by name and resolved to its spectral type via SIMBAD, which selects
 * the closest template from the Pickles (1998) library; the template can also be picked
 * directly from the list. The chosen template key is returned by selectedTemplate() (empty
 * when "None" is selected).
 */
class CameraOpticalSpectrumReferenceDialog : public QDialog
{
    Q_OBJECT
public:
    explicit CameraOpticalSpectrumReferenceDialog(const QString& currentTemplate, QWidget* parent = nullptr);

    [[nodiscard]] QString selectedTemplate() const;

private:
    void lookUpObject();
    void handleLookupReply(QNetworkReply* reply);
    void selectTemplateKey(const QString& key);

    QLineEdit* m_nameEdit;
    QPushButton* m_lookUpButton;
    QLabel* m_resultLabel;
    QComboBox* m_templateCombo;
    QNetworkAccessManager* m_networkManager;
};

#endif // INCLUDE_FEATURE_CAMERAOPTICALSPECTRUMREFERENCEDIALOG_H_
