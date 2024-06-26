///////////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2020 Edouard Griffiths, F4EXB <f4exb06@gmail.com>                   //
//                                                                                   //
// This program is free software; you can redistribute it and/or modify              //
// it under the terms of the GNU General Public License as published by              //
// the Free Software Foundation as version 3 of the License, or                      //
// (at your option) any later version.                                               //
//                                                                                   //
// This program is distributed in the hope that it will be useful,                   //
// but WITHOUT ANY WARRANTY; without even the implied warranty of                    //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                      //
// GNU General Public License V3 for more details.                                   //
//                                                                                   //
// You should have received a copy of the GNU General Public License                 //
// along with this program. If not, see <http://www.gnu.org/licenses/>.              //
///////////////////////////////////////////////////////////////////////////////////////
#include "recordinfodialog.h"
#include "ui_recordinfodialog.h"

RecordInfoDialog::RecordInfoDialog(const QString& text, QWidget* parent) :
	QDialog(parent),
	ui(new Ui::RecordInfoDialog)
{
	ui->setupUi(this);
    ui->infoText->setText(text);
}

RecordInfoDialog::~RecordInfoDialog()
{
	delete ui;
}
