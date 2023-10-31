/********************************************************************************
** Form generated from reading UI file 'moobot_hmi.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MOOBOT_HMI_H
#define UI_MOOBOT_HMI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Moobot_hmi
{
public:
    QWidget *centralWidget;

    void setupUi(QMainWindow *Moobot_hmi)
    {
        if (Moobot_hmi->objectName().isEmpty())
            Moobot_hmi->setObjectName(QString::fromUtf8("Moobot_hmi"));
        Moobot_hmi->setWindowModality(Qt::NonModal);
        Moobot_hmi->resize(1005, 711);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(Moobot_hmi->sizePolicy().hasHeightForWidth());
        Moobot_hmi->setSizePolicy(sizePolicy);
        Moobot_hmi->setStyleSheet(QString::fromUtf8(""));
        centralWidget = new QWidget(Moobot_hmi);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(centralWidget->sizePolicy().hasHeightForWidth());
        centralWidget->setSizePolicy(sizePolicy1);
        Moobot_hmi->setCentralWidget(centralWidget);

        retranslateUi(Moobot_hmi);

        QMetaObject::connectSlotsByName(Moobot_hmi);
    } // setupUi

    void retranslateUi(QMainWindow *Moobot_hmi)
    {
        Moobot_hmi->setWindowTitle(QApplication::translate("Moobot_hmi", "Moobot_hmi", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Moobot_hmi: public Ui_Moobot_hmi {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MOOBOT_HMI_H
