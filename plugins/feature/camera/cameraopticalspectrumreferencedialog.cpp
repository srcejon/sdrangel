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

#include <QComboBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QPushButton>
#include <QVBoxLayout>

#include "gui/dialogpositioner.h"

#include "cameraopticalspectrumlibrary.h"
#include "cameraopticalspectrumreferencedialog.h"

namespace {
// CDS puts browser-like user agents behind a proof-of-work anti-bot challenge, which a
// plain request cannot answer; identifying as SDRangel is passed straight through.
const QByteArray kUserAgent = QByteArrayLiteral("SDRangel (+https://github.com/f4exb/sdrangel)");
}

CameraOpticalSpectrumReferenceDialog::CameraOpticalSpectrumReferenceDialog(const QString& currentTemplate, QWidget* parent)
    : QDialog(parent),
      m_networkManager(new QNetworkAccessManager(this))
{
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
    // Without a timeout a hung connection leaves the Look up button disabled forever
    m_networkManager->setTransferTimeout(15000);
#endif
    setWindowTitle(tr("Reference spectrum"));
    resize(430, 200);
    new DialogPositioner(this, true);

    m_nameEdit = new QLineEdit(this);
    m_nameEdit->setPlaceholderText(tr("e.g. Vega, Betelgeuse, HD 172167"));
    m_nameEdit->setToolTip(tr("Name of a star to look up in SIMBAD to determine its spectral type"));
    m_lookUpButton = new QPushButton(tr("Look up"), this);
    m_lookUpButton->setToolTip(tr("Query SIMBAD for the spectral type of the named star"));
    m_lookUpButton->setDefault(false);
    m_lookUpButton->setAutoDefault(false);

    m_resultLabel = new QLabel(this);
    m_resultLabel->setWordWrap(true);

    m_templateCombo = new QComboBox(this);
    m_templateCombo->setToolTip(tr("Spectral type template from the Pickles (1998) stellar library, or an SDSS QSO/galaxy\n"
        "composite for emission objects (rest frame; set the Redshift z control to shift it)"));
    m_templateCombo->addItem(tr("None"), QString());
    for (const CameraOpticalSpectrumTemplate& entry : CameraOpticalSpectrumLibrary::templates()) {
        m_templateCombo->addItem(entry.m_name, entry.m_key);
    }
    m_templateCombo->insertSeparator(m_templateCombo->count());
    for (const CameraOpticalSpectrumEmissionTemplate& entry : CameraOpticalSpectrumLibrary::emissionTemplates()) {
        m_templateCombo->addItem(entry.m_name, entry.m_key);
    }
    selectTemplateKey(currentTemplate);

    connect(m_lookUpButton, &QPushButton::clicked, this, [this]() { lookUpObject(); });
    connect(m_nameEdit, &QLineEdit::returnPressed, this, [this]() { lookUpObject(); });
    connect(m_networkManager, &QNetworkAccessManager::finished, this, [this](QNetworkReply* reply) {
        handleLookupReply(reply);
    });

    auto* nameLayout = new QHBoxLayout();
    nameLayout->addWidget(m_nameEdit, 1);
    nameLayout->addWidget(m_lookUpButton);

    auto* form = new QFormLayout();
    form->addRow(tr("Star"), nameLayout);
    form->addRow(QString(), m_resultLabel);
    form->addRow(tr("Spectral type"), m_templateCombo);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    auto* note = new QLabel(tr("Stellar spectra are from the Pickles (1998) library (CDS J/PASP/110/863); QSO and galaxy "
                               "composites are from the SDSS DR7 spectral templates. All are downloaded on demand and cached."), this);
    note->setWordWrap(true);

    auto* layout = new QVBoxLayout();
    layout->addLayout(form);
    layout->addWidget(note);
    layout->addStretch();
    layout->addWidget(buttons);
    setLayout(layout);
}

QString CameraOpticalSpectrumReferenceDialog::selectedTemplate() const
{
    return m_templateCombo->currentData().toString();
}

void CameraOpticalSpectrumReferenceDialog::selectTemplateKey(const QString& key)
{
    const int index = m_templateCombo->findData(key);
    m_templateCombo->setCurrentIndex((index >= 0) ? index : 0);
}

void CameraOpticalSpectrumReferenceDialog::lookUpObject()
{
    const QString name = m_nameEdit->text().trimmed();
    if (name.isEmpty()) {
        return;
    }

    m_lookUpButton->setEnabled(false);
    m_resultLabel->setText(tr("Looking up %1...").arg(name));

    QNetworkRequest request{QUrl(CameraOpticalSpectrumLibrary::simbadLookupUrl(name))};
    request.setHeader(QNetworkRequest::UserAgentHeader, kUserAgent);
    request.setAttribute(QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::NoLessSafeRedirectPolicy);
    m_networkManager->get(request);
}

void CameraOpticalSpectrumReferenceDialog::handleLookupReply(QNetworkReply* reply)
{
    reply->deleteLater();
    m_lookUpButton->setEnabled(true);

    if (reply->error() != QNetworkReply::NoError)
    {
        m_resultLabel->setText(tr("SIMBAD lookup failed: %1").arg(reply->errorString()));
        return;
    }

    QString mainId;
    QString spectralType;
    if (!CameraOpticalSpectrumLibrary::parseSimbadResponse(reply->readAll(), mainId, spectralType))
    {
        m_resultLabel->setText(tr("SIMBAD has no spectral type for '%1'").arg(m_nameEdit->text().trimmed()));
        return;
    }

    const QString key = CameraOpticalSpectrumLibrary::matchTemplate(spectralType);
    if (key.isEmpty())
    {
        // Non-MK types (white dwarfs, Wolf-Rayet, carbon stars) have no Pickles template
        m_resultLabel->setText(tr("%1 is type %2, which has no template in the library").arg(mainId, spectralType));
        return;
    }

    selectTemplateKey(key);
    const CameraOpticalSpectrumTemplate* entry = CameraOpticalSpectrumLibrary::findTemplate(key);
    m_resultLabel->setText(tr("%1 is type %2 - using the %3 template")
        .arg(mainId, spectralType, entry ? entry->m_name : key));
}
